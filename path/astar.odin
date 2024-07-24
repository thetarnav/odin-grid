package grid_path

import "base:runtime"

import "core:slice"
import "core:math"

import grid ".."


Astar :: struct {
	walls        : grid.Grid(bool),
	// cells to move from initial position
	costs_g      : grid.Grid(f32),
	// distance to goal position
	costs_f      : grid.Grid(f32),
	// posititon that you came from
	came_from    : grid.Grid(grid.Coord),
	// priority queue, with last being the smallest
	open_list_buf: [^]Pos_With_Cost,
	open_list_len: int,
	init, goal   : grid.Coord,
}

Pos_With_Cost :: struct {
	pos : grid.Coord,
	cost: f32,
}


heuristic :: grid.distance

astar_init :: proc (
	a         : ^Astar,
	walls     : grid.Grid(bool),
	init, goal: grid.Coord,
	allocator := context.allocator,
	loc       := #caller_location,
) -> (
	err: runtime.Allocator_Error,
) {
	if grid.len(walls) == 0 {
		return
	}

	a.goal      = goal
	a.init      = init
	a.walls     = walls
	a.costs_g   = grid.make(f32       , walls.size, allocator, loc) or_return
	a.costs_f   = grid.make(f32       , walls.size, allocator, loc) or_return
	a.came_from = grid.make(grid.Coord, walls.size, allocator, loc) or_return

	grid.fill(&a.costs_g, -1)

	init_f_cost := heuristic(init, goal)

	grid.set(&a.costs_g, init, 0)
	grid.set(&a.costs_f, init, init_f_cost)

	open_list      := make([]Pos_With_Cost, grid.len(walls), allocator, loc) or_return
	open_list[0]    = {init, init_f_cost}
	a.open_list_buf = raw_data(open_list)
	a.open_list_len = 1

	return
}

@require_results
astar_make :: proc (
	walls     : grid.Grid(bool),
	init, goal: grid.Coord,
	allocator := context.allocator,
	loc       := #caller_location,
) -> (
	a: Astar,
	err: runtime.Allocator_Error,
) #optional_allocator_error
{
	err = astar_init(&a, walls, init, goal, allocator, loc)
	return
}

open_list_pop :: proc (a: ^Astar) -> (pos: grid.Coord, ok: bool) {
	if a.open_list_len > 0 {
		a.open_list_len -= 1
		pos = a.open_list_buf[a.open_list_len].pos
		ok = true
	}
	return
}

open_list_push :: proc (a: ^Astar, item: Pos_With_Cost) {

	index, _ := slice.binary_search_by(a.open_list_buf[:a.open_list_len], item, high_to_low_compare)

	copy(a.open_list_buf[index+1:a.open_list_len+1], a.open_list_buf[index:a.open_list_len])

	a.open_list_buf[index] = item
	a.open_list_len       += 1

	high_to_low_compare :: proc (a, b: Pos_With_Cost) -> slice.Ordering {
		return slice.cmp(b.cost, a.cost)
	}
}

astar_add_step :: proc (a: ^Astar, from, to: grid.Coord, cost: f32) {

	cost_g_old := grid.get(a.costs_g, to)
	cost_g_new := grid.get(a.costs_g, from) + cost

	if cost_g_old == -1 || cost_g_new < cost_g_old {

		cost_f := cost_g_new + heuristic(to, a.goal)

		grid.set(&a.came_from, to, from)
		grid.set(&a.costs_g,   to, cost_g_new)
		grid.set(&a.costs_f,   to, cost_f)

		open_list_push(a, {to, cost_f})
	}
}

astar_reconstruct_path :: proc (path: ^[dynamic]grid.Coord, a: Astar, loc := #caller_location) {

	p := a.goal
	for p != a.init {
		append(path, p, loc=loc)
		p = grid.get(a.came_from, p)
	}

	slice.reverse(path[:])
}


astar :: proc (
	path      : ^[dynamic]grid.Coord,
	walls     : grid.Grid(bool),
	init, goal: grid.Coord,
	allocator := context.temp_allocator,
	loc       := #caller_location,
) -> bool {

	a := astar_make(walls, init, goal, allocator, loc)

	for current in open_list_pop(&a) {

		if current == goal {
			astar_reconstruct_path(path, a, loc)
			return true
		}

		DIRECTIONS :: [8]Pos_With_Cost{
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
			neighbor := current + d.pos

			if grid.inside(walls, neighbor) && !grid.get(walls, neighbor) {
				astar_add_step(&a, current, neighbor, d.cost)
			}
		}
	}

	return false
}


/*
Jump Point Search
*/
jps :: proc (
	path      : ^[dynamic]grid.Coord,
	walls     : grid.Grid(bool),
	init, goal: grid.Coord,
	allocator := context.temp_allocator,
	loc       := #caller_location,
) -> bool {

	a := astar_make(walls, init, goal, allocator, loc)

	for current in open_list_pop(&a) {

		if current == goal {
			astar_reconstruct_path(path, a, loc)
			return true
		}

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
			if p, ok := jump(walls, current, d, goal).?; ok {
				astar_add_step(&a, current, p, heuristic(current, p))
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
