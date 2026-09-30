package grid

import "core:math/linalg"


Coord :: [2]int

view          :: proc {grid_view, grid_view_till_end, grid_view_whole}
view_safe     :: proc {grid_view_safe, grid_view_till_end_safe}

to_idx        :: proc {grid_to_idx, view_to_idx}
idx           :: to_idx
to_idx_safe   :: proc {grid_to_idx_safe, view_to_idx_safe}

to_x          :: proc {grid_to_x, view_to_x}
to_y          :: proc {grid_to_y, view_to_y}
to_xy         :: proc {grid_to_xy, view_to_xy}
to_coord      :: to_xy
coord         :: to_xy

get           :: proc {grid_get, view_get}
get_safe      :: proc {grid_get_safe, view_get_safe}

ptr           :: proc {grid_ptr, view_ptr}
ptr_safe      :: proc {grid_ptr_safe, view_ptr_safe}
ptr_idx       :: proc {grid_ptr_idx, view_ptr_idx}
ptr_idx_safe  :: proc {grid_ptr_idx_safe, view_ptr_idx_safe}

set           :: proc {grid_set, view_set}
set_safe      :: proc {grid_set_safe, view_set_safe}

set_idx       :: proc {grid_set_idx, view_set_idx}
set_idx_safe  :: proc {grid_set_idx_safe, view_set_idx_safe}

inside        :: proc {grid_inside, view_inside}
inside_idx    :: proc {grid_inside_idx, view_inside_idx}
in_bounds     :: inside
in_bounds_idx :: inside_idx

size          :: proc {grid_size, view_size}
len           :: proc {grid_len, view_len}
slice         :: proc {grid_slice, view_slice}
zero          :: proc {grid_zero, view_zero}
fill          :: proc {grid_fill, view_fill}

@require_results
distance :: proc (a, b: Coord) -> f32 {
	return linalg.distance(([2]f32)(a), ([2]f32)(b))
}

@require_results
manhattan_distance :: proc (a, b: Coord) -> int {
	return abs(a.x - b.x) + abs(a.y - b.y)
}

@require_results
are_diagonal :: proc (a, b: Coord) -> bool {
	return abs(a.x - b.x) == abs(a.y - b.y)
}

@require_results
next_surrounding_cell :: proc "contextless" (#no_broadcast p: Coord) -> Coord {

	l := max(abs(p.x), abs(p.y))
	f := abs(abs(p.x) - abs(p.y))
	d := l-f

	switch p {
	case { d,  l}: return {-l, -d-1} if f > 0 else {-l-1, 0}
	case { d, -l}: return { d,  l}
	case {-d,  l}: return { d, -l}
	case {-d, -l}: return {-d,  l}
	case { l,  d}: return {-d, -l}
	case { l, -d}: return { l,  d}
	case {-l,  d}: return { l, -d}
	case {-l, -d}: return {-l,  d}
	}

	unreachable()
}

/*
	NW N NE
	 W    E
	SW S SE
*/
Direction :: enum u8 {
	N,   E, S,   W,
	NE, SE, SW, NW,
}

DIRECTIONS            :: [8]Direction{
	.N,   .E, .S,   .W,
	.NE, .SE, .SW, .NW,
}
DIRECTIONS_ORTHOGONAL :: [4]Direction{
	.N,   .E, .S,   .W,
}
DIRECTIONS_DIAGNOAL   :: [4]Direction{
	.NE, .SE, .SW, .NW,
}

DIRECTION_VECTORS :: [Direction][2]int{
	.N  = { 0, -1},
	.E  = { 1,  0},
	.S  = { 0,  1},
	.W  = {-1,  0},
	.NE = { 1, -1},
	.SE = { 1,  1},
	.SW = {-1,  1},
	.NW = {-1, -1},
}

