package grid

@require import "base:runtime"

@require import slice_pkg "core:slice"


Grid_View :: struct ($T: typeid) {
	grid:     Grid(T),
	pos, end: int,
}

@require_results
view_to_idx :: #force_inline proc "contextless" (grid: Grid_View($T), #no_broadcast p: Coord) -> int {
	return p.x + p.y * grid.x
}
view_idx :: to_idx
@require_results
view_to_idx_safe :: #force_inline proc "contextless" (grid: Grid_View($T), #no_broadcast p: Coord) -> (i: int, ok: bool) {
	return p.x + p.y * grid.x, inside(grid, p)
}
view_idx_safe :: to_idx_safe

@require_results
view_to_x :: #force_inline proc "contextless" (view: Grid_View($T), #any_int i: int) -> int {
	return i % size(view).x
}
@require_results
view_to_y :: #force_inline proc "contextless" (view: Grid_View($T), #any_int i: int) -> int {
	return i / size(view).x
}
@require_results
view_to_xy :: #force_inline proc "contextless" (view: Grid_View($T), #any_int i: int) -> (p: Coord) {
	return {i % size(view).x, i / size(view).x}
}
view_to_coord :: to_xy
view_coord    :: to_xy

@require_results
view_get :: #force_inline proc "contextless" (view: Grid_View($T), #no_broadcast p: Coord, loc := #caller_location) -> T #no_bounds_check {
	runtime.bounds_check_error_loc(loc, p.x, size(view).x)
	runtime.bounds_check_error_loc(loc, p.y, size(view).y)
	return get(view.grid, view_pos(view)+p)
}
@require_results
view_get_safe :: #force_inline proc "contextless" (view: Grid_View($T), #no_broadcast p: Coord) -> (cell: T, ok: bool) {
	inside(view, p) or_return
	return get_safe(view.grid, view_pos(view)+p)
}

@require_results
view_ptr :: #force_inline proc "contextless" (view: ^Grid_View($T), #no_broadcast p: Coord, loc := #caller_location) -> ^T #no_bounds_check {
	runtime.bounds_check_error_loc(loc, p.x, size(view^).x)
	runtime.bounds_check_error_loc(loc, p.y, size(view^).y)
	return ptr(&view.grid, view_pos(view^)+p)
}
@require_results
view_ptr_safe :: #force_inline proc "contextless" (view: ^Grid_View($T), #no_broadcast p: Coord) -> (cell: ^T, ok: bool) {
	inside(view, p) or_return
	return ptr_safe(&view.grid, view_pos(view^)+p)
}
@require_results
view_ptr_idx :: #force_inline proc "contextless" (grid: ^Grid_View($T), #any_int i: int, loc := #caller_location) -> ^T #no_bounds_check {
	runtime.bounds_check_error_loc(loc, i, len(grid^))
	return &grid.data[i]
}
@require_results
view_ptr_idx_safe :: #force_inline proc "contextless" (grid: ^Grid_View($T), #any_int i: int) -> (cell: ^T, ok: bool) {
	inside_idx(grid^, i) or_return
	return &grid.data[i], true
}

view_set :: #force_inline proc "contextless" (view: ^Grid_View($T), #no_broadcast p: Coord, v: T, loc := #caller_location) #no_bounds_check {
	runtime.bounds_check_error_loc(loc, p.x, size(view^).x)
	runtime.bounds_check_error_loc(loc, p.y, size(view^).y)
	set(&view.grid, view_pos(view^)+p, v)
}
view_set_safe :: #force_inline proc "contextless" (view: ^Grid_View($T), #no_broadcast p: Coord, v: T) -> (ok: bool) {
	inside(view, p) or_return
	return set_safe(&view.grid, view_pos(view^)+p, v)
}

view_set_idx :: #force_inline proc "contextless" (grid: ^Grid_View($T), #any_int i: int, v: T, loc := #caller_location) #no_bounds_check {
	runtime.bounds_check_error_loc(loc, i, len(grid^))
	grid.data[i] = v
}
view_set_idx_safe :: #force_inline proc "contextless" (grid: ^Grid_View($T), #any_int i: int, v: T) -> (ok: bool) {
	(i >= 0 && i < len(grid^)) or_return
	grid.data[i] = v
}

@require_results
view_inside :: #force_inline proc "contextless" (view: Grid_View($T), #no_broadcast pos: Coord) -> bool {
	p, s := view_pos(view), view_size(view)
	return uint(pos.x-p.x) < uint(s.x) && uint(pos.y-p.y) < uint(s.y)
}
@require_results
view_inside_idx :: #force_inline proc "contextless" (view: Grid_View($T), #any_int idx: int) -> bool {
	return uint(idx) < uint(len(grid))
}
view_in_bounds     :: inside
view_in_bounds_idx :: inside_idx

@require_results
view_x :: #force_inline proc "contextless" (view: Grid_View($T)) -> int {
	return coord(view.grid, view.pos).x
}
@require_results
view_y :: #force_inline proc "contextless" (view: Grid_View($T)) -> int {
	return coord(view.grid, view.pos).y
}
@require_results
view_pos :: #force_inline proc "contextless" (view: Grid_View($T)) -> Coord {
	return coord(view.grid, view.pos)
}

@require_results
view_size :: #force_inline proc "contextless" (view: Grid_View($T)) -> [2]int {
	return coord(view.grid, view.end-1) + 1 - coord(view.grid, view.pos)
}

@require_results
view_len :: #force_inline proc "contextless" (grid: Grid_View($T)) -> int {
	return grid.x*grid.y
}

@require_results
view_slice :: #force_inline proc "contextless" (view: Grid_View($T)) -> []T {
	return slice(view.grid)[view.pos:view.end]
}

view_zero :: proc (grid: ^Grid_View($T)) {
	slice_pkg.zero(slice(grid^))
}

view_fill :: proc (grid: ^Grid_View($T), v: T) {
	slice_pkg.fill(slice(grid^), v)
}
