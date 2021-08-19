
#pragma once



template <class T>
class no_init_alloc : public std::allocator<T>
{
public:
//    using std::allocator<T>::allocator;

    template <class U, class... Args> void construct(U*, Args&&...) {}
};

