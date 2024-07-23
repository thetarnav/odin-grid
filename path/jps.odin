package grid_path

import "core:slice"

import grid ".."


jps :: proc (
	path      : ^[dynamic]grid.Coord,
	grid_walls: grid.Grid(bool),
	init, goal: grid.Coord,
) -> bool {

	heuristic :: grid.distance

	Pos_With_Cost :: struct {p: grid.Coord, cost: f32}

	/*
	g cost - cells to move from initial position
	f cost - distance to goal position
	from   - posititon that you came from
	*/
	grid_g    := grid.make(f32       , grid_walls.size, context.temp_allocator)
	grid_f    := grid.make(f32       , grid_walls.size, context.temp_allocator)
	grid_from := grid.make(grid.Coord, grid_walls.size, context.temp_allocator)

	grid.fill(&grid_g, -1)

	init_f_cost := heuristic(init, goal)

	grid.set(&grid_g, init, 0)
	grid.set(&grid_f, init, init_f_cost)

	open_list := make([dynamic]Pos_With_Cost, 0, grid.len(grid_walls)/2, context.temp_allocator)

	append(&open_list, Pos_With_Cost{init, init_f_cost})

	for len(open_list) > 0 {

		current := pop(&open_list).p // pop one with lowest f cost

		if current == goal {

			/* Reconstruct the path by backtracking */
			p := current
			for p != init {
				append(path, p)
				p = grid.get(grid_from, p)
			}

			slice.reverse(path[:])

			return true
		}

		/*
		Jump Point Search
		*/

		DIRECTIONS :: [8]grid.Coord{
			{-1,  0},
			{ 1,  0},
			{ 0, -1},
			{ 0,  1},
			{-1, -1},
			{ 1, -1},
			{-1,  1},
			{ 1,  1},
		}

		for d in DIRECTIONS {
			
			p := jump(grid_walls, current, d, goal).? or_continue
			
			cost_g_old := grid.get(grid_g, p)
			cost_g_new := grid.get(grid_g, current) + heuristic(current, p)

			if cost_g_old == -1 || cost_g_new < cost_g_old {

				cost_f := cost_g_new + heuristic(p, goal)

				grid.set(&grid_from, p, current)
				grid.set(&grid_g,    p, cost_g_new)
				grid.set(&grid_f,    p, cost_f)

				insert_ordered_by(&open_list, Pos_With_Cost{p, cost_f}, high_to_low_compare)

				high_to_low_compare :: proc (a, b: Pos_With_Cost) -> slice.Ordering {
					return slice.cmp(b.cost, a.cost)
				}
			}
		}
	}

	jump :: proc (walls: grid.Grid(bool), p, d, goal: grid.Coord) -> Maybe(grid.Coord) {
		
		for p := p+d;; p += d {

			if !grid.inside(walls, p) || grid.get(walls, p) {
				return nil
			}

			if p == goal {
				return p
			}

			switch d {
			case {-1, -1},
			     { 1, -1},
			     {-1,  1},
			     { 1,  1}:
				// +---+---+---+
				// | ↘ |   |   |
				// +---+---+---+
				// | W | P |   |
				// +---+---+---+
				// | E | ↓ |   |
				// +---+---+---+
				if ((grid.inside(walls, {p.x-d.x, p.y+d.y})  &&
				     grid.get   (walls, {p.x-d.x, p.y    })  &&
				    !grid.get   (walls, {p.x-d.x, p.y+d.y})) ||
				// +---+---+---+
				// | ↘ | W | E |
				// +---+---+---+
				// |   | P | → |
				// +---+---+---+
				// |   |   |   |
				// +---+---+---+
				   ((grid.inside(walls, {p.x+d.x, p.y-d.y})  &&
				     grid.get   (walls, {p.x    , p.y-d.y})  &&
				    !grid.get   (walls, {p.x+d.x, p.y-d.y})) ||
				   /* expand vertically */
				   jump(walls, p, {0, d.y}, goal) != nil) ||
				   jump(walls, p, {d.x, 0}, goal) != nil)
				{
					return p
				}

			case {-1,  0},
			     { 1,  0}:
				// +---+---+---+
				// |   | W | E |
				// +---+---+---+
				// | → | P | → |
				// +---+---+---+
				// |   | W | E |
				// +---+---+---+
				if /* UP */
				   (grid.inside(walls, {p.x+d.x, p.y+1})  &&
				    grid.get   (walls, {p.x    , p.y+1})  &&
				   !grid.get   (walls, {p.x+d.x, p.y+1})) ||
				   /* DOWN */
				   (grid.inside(walls, {p.x+d.x, p.y-1})  &&
				    grid.get   (walls, {p.x    , p.y-1})  &&
				   !grid.get   (walls, {p.x+d.x, p.y-1}))
				{
					return p 
				}

			case { 0, -1},
			     { 0,  1}:
				// +---+---+---+
				// |   | ↓ |   |
				// +---+---+---+
				// | W | P | W |
				// +---+---+---+
				// | E | ↓ | E |
				// +---+---+---+
				if /* RIGHT */
				   (grid.inside(walls, {p.x+1, p.y+d.y})  &&
				    grid.get   (walls, {p.x+1, p.y    })  &&
				   !grid.get   (walls, {p.x+1, p.y+d.y})) ||
				   /* LEFT */
				   (grid.inside(walls, {p.x-1, p.y+d.y})  &&
				    grid.get   (walls, {p.x-1, p.y    })  &&
				   !grid.get   (walls, {p.x-1, p.y+d.y}))
				{
					return p
				}
			}
		}
	}

	return false
}
