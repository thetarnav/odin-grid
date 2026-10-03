package grid

import           "base:builtin"
import           "base:runtime"
import slice_pkg "core:slice"

_ :: runtime
_ :: slice_pkg


Grid_Static :: struct ($X: int, $Y: int, $T: typeid) {
	data: [X * Y]T,
}

@require_results
static_grid :: proc "contextless" (grid: ^Grid_Static($X, $Y, $T)) -> Grid(T) {
	return {cast([^]T)&grid.data, {X, Y}}
}

@require_results
static_view :: proc "contextless" (grid: ^Grid_Static($X, $Y, $T), pos, size: Coord, loc := #caller_location) -> Grid_View(T) {
	runtime.bounds_check_error_loc(loc, pos.x, X)
	runtime.bounds_check_error_loc(loc, pos.y, Y)
	runtime.bounds_check_error_loc(loc, pos.x+size.x-1, X)
	runtime.bounds_check_error_loc(loc, pos.y+size.y-1, Y)
	return {static_grid(grid), idx(grid^, pos), idx(grid^, pos+size-1) + 1 - idx(grid^, pos)}
}
@require_results
static_view_till_end :: proc "contextless" (grid: ^Grid_Static($X, $Y, $T), pos: Coord, loc := #caller_location) -> Grid_View(T) {
	runtime.bounds_check_error_loc(loc, pos.x, X)
	runtime.bounds_check_error_loc(loc, pos.y, Y)
	return {static_grid(grid), idx(grid^, pos), len(grid)-idx(grid^, pos)}
}
@require_results
static_view_whole :: proc "contextless" (grid: ^Grid_Static($X, $Y, $T)) -> Grid_View(T) {
	return {static_grid(grid), 0, len(grid^)}
}

@require_results
static_view_safe :: proc "contextless" (grid: ^Grid_Static($X, $Y, $T), pos, size: Coord, loc := #caller_location) -> (view: Grid_View(T), ok: bool) {
	inside(grid^, pos) or_return
	inside(grid^, pos+size-1) or_return
	return {grid, idx(grid^, pos), idx(grid^, pos+size-1) + 1 - idx(grid^, pos)}, true
}
@require_results
static_view_till_end_safe :: proc "contextless" (grid: ^Grid_Static($X, $Y, $T), pos: Coord, loc := #caller_location) -> (view: Grid_View(T), ok: bool) {
	inside(grid^, pos) or_return
	return {grid^, idx(grid^, pos), len(grid^)-idx(grid^, pos)}, true
}

@require_results
static_to_idx :: #force_inline proc "contextless" (grid: Grid_Static($X, $Y, $T), #no_broadcast p: Coord) -> int {
	return p.x + p.y * X
}
static_idx :: to_idx
@require_results
static_to_idx_safe :: #force_inline proc "contextless" (grid: Grid_Static($X, $Y, $T), #no_broadcast p: Coord) -> (i: int, ok: bool) {
	return p.x + p.y * X, inside(grid, p)
}
static_idx_safe :: to_idx_safe

@require_results
static_to_x :: #force_inline proc "contextless" (grid: Grid_Static($X, $Y, $T), #any_int i: int) -> int {
	return i % grid.x
}
@require_results
static_to_y :: #force_inline proc "contextless" (grid: Grid_Static($X, $Y, $T), #any_int i: int) -> int {
	return i / grid.x
}
@require_results
static_to_xy :: #force_inline proc "contextless" (grid: Grid_Static($X, $Y, $T), #any_int i: int) -> (p: Coord) {
	return {i % grid.x, i / grid.size.x}
}
static_to_coord :: to_xy
static_coord    :: to_xy

@require_results
static_get :: #force_inline proc "contextless" (grid: Grid_Static($X, $Y, $T), #no_broadcast p: Coord, loc := #caller_location) -> T #no_bounds_check {
	runtime.bounds_check_error_loc(loc, p.x, grid.x)
	runtime.bounds_check_error_loc(loc, p.y, grid.y)
	return grid.data[p.x + p.y * grid.x]
}
@require_results
static_get_safe :: #force_inline proc "contextless" (grid: Grid_Static($X, $Y, $T), #no_broadcast p: Coord) -> (cell: T, ok: bool) {
	inside(grid, p) or_return
	return grid.data[p.x + p.y * grid.x], true
}

@require_results
static_ptr :: #force_inline proc "contextless" (grid: ^Grid_Static($X, $Y, $T), #no_broadcast p: Coord, loc := #caller_location) -> ^T #no_bounds_check {
	runtime.bounds_check_error_loc(loc, p.x, grid.x)
	runtime.bounds_check_error_loc(loc, p.y, grid.y)
	return &grid.data[p.x + p.y * grid.x]
}
@require_results
static_ptr_safe :: #force_inline proc "contextless" (grid: ^Grid_Static($X, $Y, $T), #no_broadcast p: Coord) -> (cell: ^T, ok: bool) {
	inside(grid^, p) or_return
	return &grid.data[p.x + p.y * grid.x], true
}
@require_results
static_ptr_idx :: #force_inline proc "contextless" (grid: ^Grid_Static($X, $Y, $T), #any_int i: int, loc := #caller_location) -> ^T #no_bounds_check {
	runtime.bounds_check_error_loc(loc, i, len(grid^))
	return &grid.data[i]
}
@require_results
static_ptr_idx_safe :: #force_inline proc "contextless" (grid: ^Grid_Static($X, $Y, $T), #any_int i: int) -> (cell: ^T, ok: bool) {
	inside_idx(grid^, i) or_return
	return &grid.data[i], true
}

static_set :: #force_inline proc "contextless" (grid: ^Grid_Static($X, $Y, $T), #no_broadcast p: Coord, v: T, loc := #caller_location) #no_bounds_check {
	runtime.bounds_check_error_loc(loc, p.x, X)
	runtime.bounds_check_error_loc(loc, p.y, Y)
	grid.data[p.x + p.y * X] = v
}
static_set_safe :: #force_inline proc "contextless" (grid: ^Grid_Static($X, $Y, $T), #no_broadcast p: Coord, v: T) -> (ok: bool) {
	inside(grid, p) or_return
	grid.data[p.x + p.y * X] = v
}

static_set_idx :: #force_inline proc "contextless" (grid: ^Grid_Static($X, $Y, $T), #any_int i: int, v: T, loc := #caller_location) #no_bounds_check {
	runtime.bounds_check_error_loc(loc, i, len(grid^))
	grid.data[i] = v
}
static_set_idx_safe :: #force_inline proc "contextless" (grid: ^Grid_Static($X, $Y, $T), #any_int i: int, v: T) -> (ok: bool) {
	(i >= 0 && i < len(grid^)) or_return
	grid.data[i] = v
}

@require_results
static_inside :: #force_inline proc "contextless" (grid: Grid_Static($X, $Y, $T), #no_broadcast p: Coord) -> bool {
	return uint(p.x) < uint(grid.x) && uint(p.y) < uint(grid.y)
}
@require_results
static_inside_idx :: #force_inline proc "contextless" (grid: Grid_Static($X, $Y, $T), #any_int idx: int) -> bool {
	return uint(idx) < uint(len(grid))
}
static_in_bounds     :: inside
static_in_bounds_idx :: inside_idx

@require_results
static_size :: #force_inline proc "contextless" (grid: Grid_Static($X, $Y, $T)) -> [2]int {
	return {X, Y}
}

@require_results
static_len :: #force_inline proc "contextless" (grid: Grid_Static($X, $Y, $T)) -> int {
	return X*Y
}

@require_results
static_slice_whole :: #force_inline proc "contextless" (grid: ^Grid_Static($X, $Y, $T)) -> []T {
	return grid.data[:X*Y]
}
@require_results
static_slice_pos_end :: #force_inline proc "contextless" (grid: ^Grid_Static($X, $Y, $T), pos, end: Coord) -> []T {
	return grid.data[idx(grid, pos):idx(grid, end)]
}
@require_results
static_slice_pos :: #force_inline proc "contextless" (grid: ^Grid_Static($X, $Y, $T), pos: Coord) -> []T {
	return grid.data[idx(grid, pos):X*Y]
}
static_slice :: proc {grid_slice_whole, grid_slice_pos_end, grid_slice_pos}

static_zero_whole :: proc (grid: ^Grid_Static($X, $Y, $T)) {
	slice_pkg.zero(slice(grid))
}
static_zero_pos_end :: proc (grid: ^Grid_Static($X, $Y, $T), pos, end: Coord) {
	slice_pkg.zero(slice(grid, pos, end))
}
static_zero_pos :: proc (grid: ^Grid_Static($X, $Y, $T), pos: Coord) {
	slice_pkg.zero(slice(grid, pos))
}
static_zero :: proc {grid_zero_whole, grid_zero_pos_end, grid_zero_pos}

static_fill_whole :: proc (grid: ^Grid_Static($X, $Y, $T), v: T) {
	slice_pkg.fill(slice(grid), v)
}
static_fill_pos_end :: proc (grid: ^Grid_Static($X, $Y, $T), pos, end: Coord, v: T) {
	slice_pkg.fill(slice(grid, pos, end), v)
}
static_fill_pos :: proc (grid: ^Grid_Static($X, $Y, $T), pos: Coord, v: T) {
	slice_pkg.fill(slice(grid, pos), v)
}
static_fill :: proc {grid_fill_whole, grid_fill_pos_end, grid_fill_pos}
