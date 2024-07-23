package grid_path

import "core:testing"

import grid ".."


@test test_jps :: proc (t: ^testing.T) {

	/*
	0     - empty
	1     - wall
	2     - start
	3..<n - path
	n     - end
	*/

	Test_Case :: struct {
		width: int,
		grid : []u8,
	}

	test_cases := []Test_Case{
		{5, { // case 0
			0, 0, 0, 0, 0,
			0, 0, 0, 0, 0,
			0, 2, 0, 3, 0,
			0, 0, 0, 0, 0,
			0, 0, 0, 0, 0,
		}},
		{5, { // case 1
			0, 0, 1, 0, 0,
			0, 0, 1, 0, 0,
			2, 0, 3, 0, 4,
			0, 0, 0, 0, 0,
			0, 0, 0, 0, 0,
		}},
		{4, { // case 2
			2, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 3,
		}},
		{5, { // case 3
			0, 0, 0, 0, 0,
			2, 0, 1, 0, 0,
			0, 0, 1, 0, 0,
			0, 0, 3, 0, 4,
			0, 0, 0, 0, 0,
		}},
		{8, { // case 4	
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 3, 0, 4, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 5,
			2, 0, 0, 0, 0, 1, 0, 6,
		}},
		{5, { // case 5
			0, 0, 0, 0, 0,
			0, 0, 1, 1, 0,
			0, 0, 1, 0, 0,
			2, 0, 1, 7, 0,
			1, 3, 1, 1, 6,
			0, 0, 4, 5, 0,
		}},
		{5, { // case 6
			0, 0, 5, 6, 0,
			0, 4, 1, 1, 7,
			0, 3, 1, 8, 0,
			2, 0, 1, 9, 0,
			1, 0, 1, 1, 1,
			0, 0, 0, 0, 0,
		}},
	}

	for test, test_i in test_cases {

		assert(len(test.grid) % test.width == 0)

		size := [2]int{test.width, len(test.grid) / test.width}

		expected := make([dynamic]grid.Coord, 0, size.x*size.y/2)
		defer delete(expected)

		walls := grid.make(bool, size)
		defer grid.delete(walls)

		init, goal: grid.Coord

		for cell, i in test.grid {
			switch cell {
			case 0:
				continue
			case 1:
				grid.set_idx(&walls, i, true)
			case 2:
				init = grid.to_xy(walls, i)
			case:
				p := grid.to_xy(walls, i)
				set_len(&expected, max(len(expected), int(cell-2)))
				#no_bounds_check expected[cell-3] = p
			}
		}

		goal = expected[len(expected)-1]

		path := make([dynamic]grid.Coord, 0, len(expected))
		defer delete(path)

		ok := jps(&path, walls, init, goal)

		testing.expectf(t,
			ok,
			"[case %d] Path was not found. Goal: {}.\n",
			test_i, goal,
		)

		testing.expectf(t,
			len(path) == len(expected),
			"[case %d] Path length is different, expected %d, got %d.\n",
			test_i, len(expected), len(path),
		)

		for p, i in path[:min(len(path), len(expected))] {
			testing.expectf(t,
				p == expected[i],
				"[case %d] [item %d] expected {}, got {}.\n",
				test_i, i+3, expected[i], p,
			)
		}
	}
}
