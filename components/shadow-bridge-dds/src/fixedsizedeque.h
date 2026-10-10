//
// Created by robolab on 31/10/23.
//

#ifndef FIXEDSIZEDEQUE_H
#define FIXEDSIZEDEQUE_H

#include <deque>

template <typename T>

class FixedSizeDeque {
public:
    FixedSizeDeque(int size) : maxSize(size) {}

    // Iterator
    using IQueue = std::deque<T>;
    typename IQueue::iterator begin()
    { return i_deque.begin(); };
    typename IQueue::iterator end()
    { return i_deque.end(); };
    typename IQueue::const_iterator begin() const
    { return i_deque.begin(); };
    typename IQueue::const_iterator end() const
    { return i_deque.begin(); };
    size_t size() const
    { return i_deque.size(); };

    void push(const T &value)
    {
        if (i_deque.size() == maxSize) {
            i_deque.pop_back();
        }
        i_deque.push_front(value);
    }

    bool full()
    {
        return i_deque.size() == maxSize;
    }

    int size()
    {
        return i_deque.size();
    }

    // Método para obtener el elemento frontal de la cola
    T front() const
    {
        if (!i_deque.empty()) {
            return i_deque.front();
        }
        throw std::runtime_error("Empty queue.");
    }

    T back() const
    {
        if (!i_deque.empty()) {
            return i_deque.back();
        }
        throw std::runtime_error("Empty queue.");
    }

    //Returns back and delete it
    T pop_back() const
    {
        if (i_deque.size() == maxSize)
        {
            auto back_aux = i_deque.back();
            return back_aux;
        }
        throw std::runtime_error("Empty queue.");
    }

    // Método para acceder a un elemento en una posición específica
    T at(size_t index) const
    {
        if (index < i_deque.size()) {
            return i_deque.at(index);
        }
        throw std::out_of_range("Índice fuera de rango.");
    }

    std::deque <T> get_queue(){
        return i_deque;
    }

    void clean_old(int pos)
    {
        if(0 <= pos and pos < i_deque.size())
        {
            int poses_to_remove = i_deque.size() - pos;
            for(int i = 0; i < poses_to_remove;i++)
            {
                i_deque.pop_back();
            }
        }
    }

private:
    size_t maxSize;
    IQueue i_deque;

//    void print() const {
//        for (const auto& val : deque) {
//            std::cout << val << " ";
//        }
//        std::cout << "\n";
//    }

};

#endif //FIXEDSIZEDEQUE_H
