package grid

import "core:math/linalg"


Coord :: [2]int

view          :: proc {grid_view, grid_view_till_end, grid_view_whole, static_view, static_view_till_end, static_view_whole}
view_safe     :: proc {grid_view_safe, grid_view_till_end_safe, static_view_safe, static_view_till_end_safe}

to_idx        :: proc {grid_to_idx,      view_to_idx,      static_to_idx}
to_idx_safe   :: proc {grid_to_idx_safe, view_to_idx_safe, static_to_idx_safe}
idx           :: to_idx

to_x          :: proc {grid_to_x,  view_to_x,  static_to_x}
to_y          :: proc {grid_to_y,  view_to_y,  static_to_y}
to_xy         :: proc {grid_to_xy, view_to_xy, static_to_xy}
to_coord      :: to_xy
coord         :: to_xy

get           :: proc {grid_get,      view_get,      static_get}
get_safe      :: proc {grid_get_safe, view_get_safe, static_get_safe}

ptr           :: proc {grid_ptr,          view_ptr,          static_ptr}
ptr_safe      :: proc {grid_ptr_safe,     view_ptr_safe,     static_ptr_safe}
ptr_idx       :: proc {grid_ptr_idx,      view_ptr_idx,      static_ptr_idx}
ptr_idx_safe  :: proc {grid_ptr_idx_safe, view_ptr_idx_safe, static_ptr_idx_safe}

set           :: proc {grid_set,      view_set,      static_set}
set_safe      :: proc {grid_set_safe, view_set_safe, static_set_safe}

set_idx       :: proc {grid_set_idx,      view_set_idx,      static_set_idx}
set_idx_safe  :: proc {grid_set_idx_safe, view_set_idx_safe, static_set_idx_safe}

inside        :: proc {grid_inside,     view_inside,     static_inside}
inside_idx    :: proc {grid_inside_idx, view_inside_idx, static_inside_idx}
in_bounds     :: inside
in_bounds_idx :: inside_idx

slice         :: proc {grid_slice_whole,   grid_slice_pos_end,   grid_slice_pos,
                       static_slice_whole, static_slice_pos_end, static_slice_pos}

size          :: proc {grid_size, view_size, static_size}
len           :: proc {grid_len,  view_len,  static_len}

zero          :: proc {grid_zero_whole,   grid_zero_pos_end,   grid_zero_pos,
                       view_zero,
                       static_zero_whole, static_zero_pos_end, static_zero_pos}

fill          :: proc {grid_fill_whole,   grid_fill_pos_end,   grid_fill_pos,
                       view_fill,
                       static_fill_whole, static_fill_pos_end, static_fill_pos}

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

