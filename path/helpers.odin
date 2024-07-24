//+private package
package grid_path

import "base:runtime"


set_len :: proc (arr: ^$A/[dynamic]$E, l: int, loc := #caller_location) -> runtime.Allocator_Error {

	if arr == nil {
		return nil
	}

	raw := cast(^runtime.Raw_Dynamic_Array)arr
	raw.len = l

	when size_of(E) == 0 {
		return nil
	}

	if raw.cap >= l {
		return nil
	}

	cap := 2 * max(runtime.DEFAULT_DYNAMIC_ARRAY_CAPACITY, l)
	return runtime._reserve_dynamic_array(raw, size_of(E), align_of(E), cap, true, loc)
}