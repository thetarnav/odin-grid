package grid_path

import "core:slice"
import "core:math"

import grid ".."


astar :: proc (
	path:       ^[dynamic]grid.Coord,
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

			p := current
			for p != init {
				append(path, p)
				p = grid.get(grid_from, p)
			}

			slice.reverse(path[:])

			return true
		}

		DIRECTIONS :: [?]Pos_With_Cost{
			{{-1,  0}, 1},
			{{ 1,  0}, 1},
			{{ 0, -1}, 1},
			{{ 0,  1}, 1},
			{{-1, -1}, math.SQRT_TWO},
			{{ 1,  1}, math.SQRT_TWO},
			{{ 1, -1}, math.SQRT_TWO},
			{{-1,  1}, math.SQRT_TWO},
		}

		for d in DIRECTIONS {

			neighbor := current + d.p

			(grid.inside(grid_walls, neighbor) && !grid.get(grid_walls, neighbor)) or_continue

			neighbor_g_cost_new := grid.get(grid_g, current) + d.cost
			neighbor_g_cost_old := grid.get(grid_g, neighbor)

			(neighbor_g_cost_old == -1 || neighbor_g_cost_new < neighbor_g_cost_old) or_continue

			neighbor_f_cost := neighbor_g_cost_new + heuristic(neighbor, goal)

			grid.set(&grid_from, neighbor, current)
			grid.set(&grid_g,    neighbor, neighbor_g_cost_new)
			grid.set(&grid_f,    neighbor, neighbor_f_cost)

			insert_ordered_by(&open_list, Pos_With_Cost{neighbor, neighbor_f_cost}, high_to_low_compare)

			high_to_low_compare :: proc (a, b: Pos_With_Cost) -> slice.Ordering {
				return slice.cmp(b.cost, a.cost)
			}
		}
	}

	return false
}
