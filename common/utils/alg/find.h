/*
MIT License

Copyright (c) 2021 Mikel Irazabal

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef FIND_ALGORITHM
#define FIND_ALGORITHM

#include <stdbool.h>
#include <stddef.h>
#include "seq_container/seq_generic.h"
#include "assoc_container/assoc_generic.h"
#include "assoc_container/bimap.h"


/*
 * It returns the first position in the data structure where the predicate i.e., param f,  returns true
 * @param ds: data structure
 * @param start_it: valid iterator position within the seq_arr_t. 
 * @param end_it: end iterator position. range [start_it, end). end needs to be reacheable from start_it. 
 * @param value: the value to compare with.
 * @param f: the predicate. Returns true if the value and the object pointed to by the iterator match. value is 
 *           the first argument and the iterator the second 
 * */

// Sequencial containers

void* find_if_arr(seq_arr_t* ds, void* start_it, void* end_it, void* value, bool (*f)(const void*, const void*));

// Associative containers

void* find_if_rb_tree(assoc_rb_tree_t* ds, void* start_it, void* end_it, void const* value, bool (*f)(const void*, const void*));

bml_iter_t find_if_bi_map_left(bi_map_t* ds, bml_iter_t start_it, bml_iter_t end_it, void const* value, bool (*f)(const void*, const void*));

bmr_iter_t find_if_bi_map_right(bi_map_t* ds, bmr_iter_t start_it, bmr_iter_t end_it, void const* value, bool (*f)(const void*, const void*));

#endif
