// Test-only preemption points around the production driver's atomic publication.
// The underlying operation remains the compiler builtin, including its ordering.
extern "C" void before_regular_publication();
extern "C" void after_regular_publication();
#define __atomic_store_n(pointer, value, order) \
    (before_regular_publication(), __atomic_store_n(pointer, value, order), after_regular_publication())
