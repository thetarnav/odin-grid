//+private package
package grid_path

import "base:intrinsics"
import "base:runtime"

import "core:slice"


insert_ordered :: proc (arr: ^$A/[dynamic]$E, e: E) \
	-> (index: int, err: runtime.Allocator_Error)
	where intrinsics.type_is_ordered(E) #optional_allocator_error
{
	return insert_ordered_by(arr, e, slice.cmp_proc(E))
}

insert_ordered_by :: proc (arr: ^$A/[dynamic]$E, e: E, cmp: proc (E, E) -> slice.Ordering) \
	-> (index: int, err: runtime.Allocator_Error) #optional_allocator_error
{
	index, _ = slice.binary_search_by(arr[:], e, cmp)
	_, err = inject_at(arr, index, e)
	return
}

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