package grid

import "base:builtin"
import "base:runtime"

@require import slice_pkg "core:slice"


Grid :: struct ($T: typeid) {
	data:       [^]T,
	using size: [2]int,
}

@require_results
make_empty :: proc (
	$T: typeid,
	size: [2]int,
	allocator := context.allocator,
	loc := #caller_location,
) -> (
	grid: Grid(T),
	error: runtime.Allocator_Error,
) #optional_allocator_error
{
	grid.size = size
	grid.data = builtin.make([^]T, size.x * size.y, allocator, loc) or_return
	return
}

@require_results
make_from_data :: proc (data: []$T, size: [2]int) -> (grid: Grid(T)) {
	assert(builtin.len(data) == size.x * size.y)
	grid.size = size
	grid.data = raw_data(data)
	return
}

make :: proc {make_empty, make_from_data}

delete :: proc (grid: Grid($T)) {
	builtin.delete(grid.data[:grid.x*grid.y])
}

@require_results
grid_view :: proc "contextless" (grid: ^Grid($T), pos, size: Coord, loc := #caller_location) -> Grid_View(T) {
	runtime.bounds_check_error_loc(loc, pos.x, grid.x)
	runtime.bounds_check_error_loc(loc, pos.y, grid.y)
	runtime.bounds_check_error_loc(loc, pos.x+size.x-1, grid.x)
	runtime.bounds_check_error_loc(loc, pos.y+size.y-1, grid.y)
	return {grid^, idx(grid^, pos), idx(grid^, pos+size-1) + 1 - idx(grid^, pos)}
}
@require_results
grid_view_till_end :: proc "contextless" (grid: ^Grid($T), pos: Coord, loc := #caller_location) -> Grid_View(T) {
	runtime.bounds_check_error_loc(loc, pos.x, grid.x)
	runtime.bounds_check_error_loc(loc, pos.y, grid.y)
	return {grid^, idx(grid^, pos), len(grid)-idx(grid^, pos)}
}
@require_results
grid_view_whole :: proc "contextless" (grid: ^Grid($T)) -> Grid_View(T) {
	return {grid^, 0, len(grid^)}
}

@require_results
grid_view_safe :: proc "contextless" (grid: ^Grid($T), pos, size: Coord, loc := #caller_location) -> (view: Grid_View(T), ok: bool) {
	inside(grid^, pos) or_return
	inside(grid^, pos+size-1) or_return
	return {grid, idx(grid^, pos), idx(grid^, pos+size-1) + 1 - idx(grid^, pos)}, true
}
@require_results
grid_view_till_end_safe :: proc "contextless" (grid: ^Grid($T), pos: Coord, loc := #caller_location) -> (view: Grid_View(T), ok: bool) {
	inside(grid^, pos) or_return
	return {grid^, idx(grid^, pos), len(grid^)-idx(grid^, pos)}, true
}

@require_results
grid_to_idx :: #force_inline proc "contextless" (grid: Grid($T), #no_broadcast p: Coord) -> int {
	return p.x + p.y * grid.x
}
grid_idx :: to_idx
@require_results
grid_to_idx_safe :: #force_inline proc "contextless" (grid: Grid($T), #no_broadcast p: Coord) -> (i: int, ok: bool) {
	return p.x + p.y * grid.x, inside(grid, p)
}
grid_idx_safe :: to_idx_safe

@require_results
grid_to_x :: #force_inline proc "contextless" (grid: Grid($T), #any_int i: int) -> int {
	return i % grid.x
}
@require_results
grid_to_y :: #force_inline proc "contextless" (grid: Grid($T), #any_int i: int) -> int {
	return i / grid.x
}
@require_results
grid_to_xy :: #force_inline proc "contextless" (grid: Grid($T), #any_int i: int) -> (p: Coord) {
	return {i % grid.x, i / grid.size.x}
}
grid_to_coord :: to_xy
grid_coord    :: to_xy

@require_results
grid_get :: #force_inline proc "contextless" (grid: Grid($T), #no_broadcast p: Coord, loc := #caller_location) -> T #no_bounds_check {
	runtime.bounds_check_error_loc(loc, p.x, grid.x)
	runtime.bounds_check_error_loc(loc, p.y, grid.y)
	return grid.data[p.x + p.y * grid.x]
}
@require_results
grid_get_safe :: #force_inline proc "contextless" (grid: Grid($T), #no_broadcast p: Coord) -> (cell: T, ok: bool) {
	inside(grid, p) or_return
	return grid.data[p.x + p.y * grid.x], true
}

@require_results
grid_ptr :: #force_inline proc "contextless" (grid: ^Grid($T), #no_broadcast p: Coord, loc := #caller_location) -> ^T #no_bounds_check {
	runtime.bounds_check_error_loc(loc, p.x, grid.x)
	runtime.bounds_check_error_loc(loc, p.y, grid.y)
	return &grid.data[p.x + p.y * grid.x]
}
@require_results
grid_ptr_safe :: #force_inline proc "contextless" (grid: ^Grid($T), #no_broadcast p: Coord) -> (cell: ^T, ok: bool) {
	inside(grid^, p) or_return
	return &grid.data[p.x + p.y * grid.x], true
}
@require_results
grid_ptr_idx :: #force_inline proc "contextless" (grid: ^Grid($T), #any_int i: int, loc := #caller_location) -> ^T #no_bounds_check {
	runtime.bounds_check_error_loc(loc, i, len(grid^))
	return &grid.data[i]
}
@require_results
grid_ptr_idx_safe :: #force_inline proc "contextless" (grid: ^Grid($T), #any_int i: int) -> (cell: ^T, ok: bool) {
	inside_idx(grid^, i) or_return
	return &grid.data[i], true
}

grid_set :: #force_inline proc "contextless" (grid: ^Grid($T), #no_broadcast p: Coord, v: T, loc := #caller_location) #no_bounds_check {
	runtime.bounds_check_error_loc(loc, p.x, grid.x)
	runtime.bounds_check_error_loc(loc, p.y, grid.y)
	grid.data[p.x + p.y * grid.x] = v
}
grid_set_safe :: #force_inline proc "contextless" (grid: ^Grid($T), #no_broadcast p: Coord, v: T) -> (ok: bool) {
	inside(grid, p) or_return
	grid.data[p.x + p.y * grid.x] = v
}

grid_set_idx :: #force_inline proc "contextless" (grid: ^Grid($T), #any_int i: int, v: T, loc := #caller_location) #no_bounds_check {
	runtime.bounds_check_error_loc(loc, i, len(grid^))
	grid.data[i] = v
}
grid_set_idx_safe :: #force_inline proc "contextless" (grid: ^Grid($T), #any_int i: int, v: T) -> (ok: bool) {
	(i >= 0 && i < len(grid^)) or_return
	grid.data[i] = v
}

@require_results
grid_inside :: #force_inline proc "contextless" (grid: Grid($T), #no_broadcast p: Coord) -> bool {
	return uint(p.x) < uint(grid.x) && uint(p.y) < uint(grid.y)
}
@require_results
grid_inside_idx :: #force_inline proc "contextless" (grid: Grid($T), #any_int idx: int) -> bool {
	return uint(idx) < uint(len(grid))
}
grid_in_bounds     :: inside
grid_in_bounds_idx :: inside_idx

@require_results
grid_size :: #force_inline proc "contextless" (grid: Grid($T)) -> [2]int {
	return grid.size
}

@require_results
grid_len :: #force_inline proc "contextless" (grid: Grid($T)) -> int {
	return grid.x*grid.y
}

@require_results
grid_slice_whole :: #force_inline proc "contextless" (grid: Grid($T)) -> []T {
	return grid.data[:grid.x*grid.y]
}
@require_results
grid_slice_pos_end :: #force_inline proc "contextless" (grid: Grid($T), pos, end: Coord) -> []T {
	return grid.data[idx(grid, pos):idx(grid, end)]
}
@require_results
grid_slice_pos :: #force_inline proc "contextless" (grid: Grid($T), pos: Coord) -> []T {
	return grid.data[idx(grid, pos):grid.x*grid.y]
}
grid_slice :: proc {grid_slice_whole, grid_slice_pos_end, grid_slice_pos}

grid_zero_whole :: proc (grid: ^Grid($T)) {
	slice_pkg.zero(slice(grid^))
}
grid_zero_pos_end :: proc (grid: ^Grid($T), pos, end: Coord) {
	slice_pkg.zero(slice(grid^, pos, end))
}
grid_zero_pos :: proc (grid: ^Grid($T), pos: Coord) {
	slice_pkg.zero(slice(grid^, pos))
}
grid_zero :: proc {grid_zero_whole, grid_zero_pos_end, grid_zero_pos}

grid_fill_whole :: proc (grid: ^Grid($T), v: T) {
	slice_pkg.fill(slice(grid^), v)
}
grid_fill_pos_end :: proc (grid: ^Grid($T), pos, end: Coord, v: T) {
	slice_pkg.fill(slice(grid^, pos, end), v)
}
grid_fill_pos :: proc (grid: ^Grid($T), pos: Coord, v: T) {
	slice_pkg.fill(slice(grid^, pos), v)
}
grid_fill :: proc {grid_fill_whole, grid_fill_pos_end, grid_fill_pos}
