package grid

import "base:builtin"
import "base:runtime"

import slice_pkg "core:slice"
import "core:math/linalg"


Grid :: struct ($T: typeid) {
	data: [^]T,
	size: [2]int,
}

Coord :: distinct [2]int

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

make_from_data :: proc (data: []$T, size: [2]int) -> (grid: Grid(T)) {
	assert(builtin.len(data) == size.x * size.y)
	grid.size = size
	grid.data = raw_data(data)
	return
}

make :: proc {make_empty, make_from_data}

delete :: proc (grid: Grid($T)) {
	builtin.delete(grid.data[:grid.size.x*grid.size.y])
}

idx :: #force_inline proc "contextless" (grid: Grid($T), p: Coord) -> int {
	return x + y * grid.size.x
}

to_x :: #force_inline proc "contextless" (grid: Grid($T), i: int) -> int {
	return i % grid.size.x
}

to_y :: #force_inline proc "contextless" (grid: Grid($T), i: int) -> int {
	return i / grid.size.x
}

to_xy :: #force_inline proc "contextless" (grid: Grid($T), i: int) -> (p: Coord) {
	return {i % grid.size.x, i / grid.size.x}
}

get :: #force_inline proc "contextless" (grid: Grid($T), p: Coord) -> T {
	return grid.data[p.x + p.y * grid.size.x]
}

ptr :: #force_inline proc "contextless" (grid: ^Grid($T), p: Coord) -> ^T {
	return &grid.data[p.x + p.y * grid.size.x]
}

set :: #force_inline proc "contextless" (grid: ^Grid($T), p: Coord, v: T) {
	grid.data[p.x + p.y * grid.size.x] = v
}

set_idx :: #force_inline proc "contextless" (grid: ^Grid($T), i: int, v: T) {
	grid.data[i] = v
}

inside :: #force_inline proc "contextless" (grid: Grid($T), p: Coord) -> bool {
	return p.x >= 0 && p.x < grid.size.x && p.y >= 0 && p.y < grid.size.y
}

len :: #force_inline proc "contextless" (grid: Grid($T)) -> int {
	return grid.size.x*grid.size.y
}

slice :: #force_inline proc "contextless" (grid: ^Grid($T)) -> []T {
	return grid.data[:grid.size.x*grid.size.y]
}

zero :: proc (grid: ^Grid($T)) {
	slice_pkg.zero(slice(grid))
}

fill :: proc (grid: ^Grid($T), v: T) {
	slice_pkg.fill(slice(grid), v)
}

distance :: proc (a, b: Coord) -> f32 {
	return linalg.distance([2]f32{f32(a.x), f32(a.y)}, [2]f32{f32(b.x), f32(b.y)})
}

manhattan_distance :: proc (a, b: Coord) -> int {
	return abs(a.x - b.x) + abs(a.y - b.y)
}

are_diagonal :: proc (a, b: Coord) -> bool {
	return abs(a.x - b.x) == abs(a.y - b.y)
}
