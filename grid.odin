package grid

import "base:builtin"
import "base:runtime"

import slice_pkg "core:slice"
import "core:math/linalg"


Grid :: struct ($T: typeid) {
	data: [^]T,
	using size: [2]int,
}

Coord :: [2]int

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
to_idx :: #force_inline proc "contextless" (grid: Grid($T), #no_broadcast p: Coord) -> int {
	return p.x + p.y * grid.x
}
idx :: to_idx
@require_results
to_idx_safe :: #force_inline proc "contextless" (grid: Grid($T), #no_broadcast p: Coord) -> (i: int, ok: bool) {
	return p.x + p.y * grid.x, inside(grid, p)
}
idx_safe :: to_idx_safe

@require_results
to_x :: #force_inline proc "contextless" (grid: Grid($T), #any_int i: int) -> int {
	return i % grid.x
}
@require_results
to_y :: #force_inline proc "contextless" (grid: Grid($T), #any_int i: int) -> int {
	return i / grid.x
}
@require_results
to_xy :: #force_inline proc "contextless" (grid: Grid($T), #any_int i: int) -> (p: Coord) {
	return {i % grid.x, i / grid.size.x}
}
to_coord :: to_xy
coord    :: to_xy

@require_results
get :: #force_inline proc "contextless" (grid: Grid($T), #no_broadcast p: Coord, loc := #caller_location) -> T #no_bounds_check {
    runtime.bounds_check_error_loc(loc, p.x, grid.x)
    runtime.bounds_check_error_loc(loc, p.y, grid.y)
	return grid.data[p.x + p.y * grid.x]
}
@require_results
get_safe :: #force_inline proc "contextless" (grid: Grid($T), #no_broadcast p: Coord) -> (cell: T, ok: bool) {
    inside(grid, p) or_return
	return grid.data[p.x + p.y * grid.x], true
}

@require_results
ptr :: #force_inline proc "contextless" (grid: ^Grid($T), #no_broadcast p: Coord, loc := #caller_location) -> ^T #no_bounds_check {
    runtime.bounds_check_error_loc(loc, p.x, grid.x)
    runtime.bounds_check_error_loc(loc, p.y, grid.y)
	return &grid.data[p.x + p.y * grid.x]
}
@require_results
ptr_safe :: #force_inline proc "contextless" (grid: ^Grid($T), #no_broadcast p: Coord) -> (cell: ^T, ok: bool) {
    inside(grid^, p) or_return
	return &grid.data[p.x + p.y * grid.x], true
}
@require_results
ptr_idx :: #force_inline proc "contextless" (grid: ^Grid($T), #any_int i: int, loc := #caller_location) -> ^T #no_bounds_check {
    runtime.bounds_check_error_loc(loc, i, len(grid^))
	return &grid.data[i]
}
@require_results
ptr_idx_safe :: #force_inline proc "contextless" (grid: ^Grid($T), #any_int i: int) -> (cell: ^T, ok: bool) {
    inside_idx(grid^, i) or_return
	return &grid.data[i], true
}

set :: #force_inline proc "contextless" (grid: ^Grid($T), #no_broadcast p: Coord, v: T, loc := #caller_location) #no_bounds_check {
    runtime.bounds_check_error_loc(loc, p.x, grid.x)
    runtime.bounds_check_error_loc(loc, p.y, grid.y)
	grid.data[p.x + p.y * grid.x] = v
}
set_safe :: #force_inline proc "contextless" (grid: ^Grid($T), #no_broadcast p: Coord, v: T) -> (ok: bool) {
    inside(grid, p) or_return
	grid.data[p.x + p.y * grid.x] = v
}

set_idx :: #force_inline proc "contextless" (grid: ^Grid($T), #any_int i: int, v: T, loc := #caller_location) #no_bounds_check {
    runtime.bounds_check_error_loc(loc, i, len(grid^))
	grid.data[i] = v
}
set_idx_safe :: #force_inline proc "contextless" (grid: ^Grid($T), #any_int i: int, v: T) -> (ok: bool) {
    (i >= 0 && i < len(grid^)) or_return
	grid.data[i] = v
}

@require_results
inside :: #force_inline proc "contextless" (grid: Grid($T), #no_broadcast p: Coord) -> bool {
	return uint(p.x) < uint(grid.x) && uint(p.y) < uint(grid.y)
}
@require_results
inside_idx :: #force_inline proc "contextless" (grid: Grid($T), #any_int idx: int) -> bool {
	return uint(idx) < uint(len(grid))
}
in_bounds     :: inside
in_bounds_idx :: inside_idx

@require_results
len :: #force_inline proc "contextless" (grid: Grid($T)) -> int {
	return grid.x*grid.y
}

@require_results
slice :: #force_inline proc "contextless" (grid: Grid($T)) -> []T {
	return grid.data[:grid.x*grid.y]
}

zero :: proc (grid: ^Grid($T)) {
	slice_pkg.zero(slice(grid))
}

fill :: proc (grid: ^Grid($T), v: T) {
	slice_pkg.fill(slice(grid^), v)
}

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

