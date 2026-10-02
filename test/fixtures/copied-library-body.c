// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Negative fixture: Box3D 47d7f7cc src/arena_allocator.c, b3CreateStack.
// This is deliberately library code, not sample mover code.
b3Stack b3CreateStack( int capacity )
{
	B3_ASSERT( capacity >= 0 );
	b3Stack stack = { 0 };
	stack.capacity = capacity;
	stack.memory = (char*)b3Alloc( capacity );
	return stack;
}
