/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_DEQUE_2D_H
#define CNOID_UTIL_DEQUE_2D_H

#include <Eigen/StdVector>
#include <memory>
#include <iterator>

namespace cnoid {

template <typename ElementType, typename Allocator = std::allocator<ElementType> >
class Deque2D
{
    typedef Deque2D<ElementType, Allocator> Deque2DType;
    
    Allocator allocator;
    ElementType* buf;
    int offset;
    int rowSize_;
    int colSize_;
    int capacity_;
    int size_;

public:

    class const_iterator : public std::iterator<std::bidirectional_iterator_tag, ElementType> {

    protected:
        ElementType* current;
        ElementType* term;
        ElementType* top;

        const_iterator(Deque2DType& owner, ElementType* pos){
            current = pos;
            top = owner.buf;
            term = top + owner.capacity;
        }
            
    public:
        const_iterator() { }

        const_iterator(const const_iterator& org) {
            current = org.current;
            term = org.term;
            top = org.top;
        }
        
        const ElementType& operator*() const {
            return *current;
        }
        void operator++() {
            ++current;
            if(current == term){
                current = top;
            }
        }
        void operator--() {
            if(current == top){
                current = term - 1;
            } else {
                --current;
            }
        }
        bool operator==(const const_iterator& rhs) const {
            return (current == rhs.current);
        }
        bool operator!=(const const_iterator& rhs) const {
            return (current != rhs.current);
        }
    };

    class iterator : public const_iterator {
    public:
        iterator() { }
        iterator(const iterator& org) : const_iterator(org) { }
        
        ElementType& operator*() {
            return *const_iterator::current;
        }
    };
        
    iterator begin() {
        return iterator(*this, buf + offset);
    }

    const_iterator const_begin() const {
        return const_iterator(*this, buf + offset);
    }

    const_iterator end() const {
        //! \todo cache this value for the performance?
        return const_iterator(*this, buf + (offset + size_) % capacity_);
    }

    class Row
    {
        ElementType* top;
        int size_;

    public:
        Row(Deque2D<ElementType, Allocator>& owner, int rowIndex) {
            size_ = owner.colSize_;
            top = owner.buf + (owner.offset + rowIndex * owner.colSize_) % owner.capacity_;
        }

        int size() const {
            return size_;
        }

        ElementType& operator[](int index){
            return top[index];
        }

        const ElementType& operator[](int index) const {
            return top[index];
        }

        ElementType* begin() {
            return top;
        }

        ElementType* end() {
            return top + size_;
        }
    };

    class Column
    {
    public:
        Column(Deque2D<ElementType, Allocator>& owner, int column) {
            top = owner.buf + column;
            offset = owner.offset_;
            colSize = owner.colSize_;
            capacity = owner.capacity_;
            rowSize = owner.rowSize_;

            end_ = iterator(*this, top + (offset + owner.size_) % owner.capacity_);
        }

        int size() const {
            return rowSize;
        }

        ElementType& operator[](int rowIndex){
            return top[(offset + rowIndex * colSize) % capacity];
        }

        const ElementType& operator[](int rowIndex) const {
            return top[(offset + rowIndex * colSize) % capacity];
        }

        class iterator : public std::iterator<std::bidirectional_iterator_tag, ElementType, int> {

            ElementType* current;
            ElementType* term;
            ElementType* top;
            int colSize;
                
        public:
                
            iterator() { }
                
            iterator(Column& column, ElementType* pos){
                current = pos;
                top = column.top;
                term = top + column.capacity;
                colSize = column.colSize;
            }
            ElementType& operator*() {
                return *current;
            }
            void operator++() {
                current += colSize;
                if(current == term){
                    current = top;
                }
            }
            void operator--() {
                if(current == top){
                    current = term - colSize;
                } else {
                    current -= colSize;
                }
            }
            bool operator==(iterator rhs) const {
                return (current == rhs.current);
            }
            bool operator!=(iterator rhs) const {
                return (current != rhs.current);
            }
        };

        iterator begin() {
            return iterator(*this, top + offset);
        }
        iterator end() {
            return end_;
        }
                    
    private:
        ElementType top;
        int offset;
        int colSize;
        int capacity;
        int rowSize;
        typename Column::iterator end_;
    };
        
    Deque2D() {
        buf = 0;
        offset = 0;
        rowSize_ = 0;
        colSize_ = 0;
        capacity_ = 0;
        size_ = 0;
    }
        
    Deque2D(int rowSize, int colSize) {

        buf = 0;
        offset = 0;
        rowSize_ = 0;
        colSize_ = 0;
        capacity_ = 0;
        size_ = 0;

        resize(rowSize_, colSize_);
    }

    Deque2D(const Deque2D<ElementType, Allocator>& org)
        : allocator(org.allocator) {

        size_ = org.size_;
        rowSize_ = org.rowSize_;
        colSize_ = org.colSize_;
        capacity_ = size_ + colSize_;
        buf = 0;

        if(capacity_){
            buf = allocator.allocate(capacity_);
            offset = 0;
            ElementType* p = buf;
            ElementType* pend = buf + size_;
            ElementType* q = org.buf + org.offset;
            ElementType* qterm = org.buf + org.capacity_;
            while(p != pend){
                allocator.construct(p++, *q++);
                if(q == qterm){
                    q = org.buf;
                }
            }
        }
    }

    Deque2DType& operator=(const Deque2DType& rhs) {
        if(this != &rhs){
            resize(rhs.rowSize_, rhs.colSize_);
            iterator p = begin();
            iterator q = rhs.begin();
            iterator qend = rhs.end();
            while(q != qend){
                *p++ = *q++;
            }
        }
        return *this;
    }
    
    virtual ~Deque2D() {
        if(buf){
            ElementType* p = buf + offset;
            const ElementType* pend = buf + (offset + size_) % capacity_;

            if(p <= pend){
                while(p != pend){
                    allocator.destroy(p++);
                }
            } else {
                for(ElementType* q = buf; q != pend; ++q){
                    allocator.destroy(q);
                }
                const ElementType* pterm = buf + capacity_;
                for(ElementType* q = p; q != pterm; ++q){
                    allocator.destroy(q);
                }
            }
            allocator.deallocate(buf, capacity_);
        }
    }

    bool empty() const {
        return !rowSize_ || !colSize_;
    }

private:
    void reallocMemory(int newColSize, int newSize, int newCapacity, bool doCopy) {

        ElementType* newBuf = allocator.allocate(newCapacity);
        ElementType* p = newBuf;
        ElementType* pend = newBuf + newSize;
                    
        if(newColSize == colSize_ && doCopy){
            // copy the existing elements
            ElementType* q = buf + offset;
            ElementType* qend = buf + (offset + size_) % capacity_;
            if(q <= qend){
                while(q != qend){
                    allocator.construct(p++, *q);
                    allocator.destroy(q++);
                }
            } else {
                for(ElementType* r = buf; r != qend; ++r){
                    allocator.construct(p++, *r);
                    allocator.destroy(r);
                }
                ElementType* qterm = buf + capacity_;
                for(ElementType* r = q; r != qterm; ++r){
                    allocator.construct(p++, *r);
                    allocator.destroy(r);
                }
            }
        } else {
            ElementType* q = buf + offset;
            ElementType* qend = buf + (offset + size_) % capacity_;
            if(q <= qend){
                while(q != qend){
                    allocator.destroy(q++);
                }
            } else {
                for(ElementType* r = buf; r != qend; ++r){
                    allocator.destroy(r);
                }
                ElementType* qterm = buf + capacity_;
                for(ElementType* r = q; r != qterm; ++r){
                    allocator.destroy(r);
                }
            }
        }
        // construct new elements
        while(p != pend){
            allocator.construct(p++, ElementType());
        }

        allocator.deallocate(buf, capacity_);
        buf = newBuf;
        capacity_ = newCapacity;
        offset = 0;
    }

    void resize(int newRowSize, int newColSize, bool doCopy) {

        const int newSize = newRowSize * newColSize;
            
        // The area for the 'end' iterator should be reserved
        const int minCapacity = newSize + newColSize;

        if(minCapacity <= capacity_){

            if((newColSize != colSize_) && (capacity_ % newColSize > 0)){
                reallocMemory(newColSize, newSize, capacity_ - (capacity_ % newColSize), doCopy);

            } else if(newSize > size_){

                ElementType* p = buf + (offset + size_) % capacity_;
                const ElementType* pend = buf + (offset + newSize) % capacity_;
                if(p <= pend){
                    while(p != pend){
                        allocator.construct(p++, ElementType());
                    }
                } else {
                    for(ElementType* r = buf; r != pend; ++r){
                        allocator.construct(r, ElementType());
                    }
                    const ElementType* pterm = buf + capacity_;
                    for(ElementType* r = p; r != pterm; ++r){
                        allocator.construct(r, ElementType());
                    }
                }
            } else if(newSize < size_){

                ElementType* p = buf + (offset + newSize) % capacity_;
                ElementType* pend = buf + (offset + size_) % capacity_;
                if(p <= pend){
                    while(p != pend){
                        allocator.destroy(p++);
                    }
                } else {
                    for(ElementType* r = buf; r != pend; ++r){
                        allocator.destroy(r);
                    }
                    const ElementType* pterm = buf + capacity_;
                    for(ElementType* r = p; r != pterm; ++r){
                        allocator.destroy(r);
                    }
                }
            }
        } else {
            if(!buf){
                capacity_ = minCapacity;
                buf = allocator.allocate(minCapacity);
                ElementType* p = buf;
                ElementType* pend = buf + newSize;
                // construct new elements
                while(p != pend){
                    allocator.construct(p++, ElementType());
                }
            } else {
                int newCapacity = newSize * 3 / 2;
                newCapacity = newCapacity - (newCapacity % newColSize) + newColSize;
                reallocMemory(newColSize, newSize, newCapacity, doCopy);
            }
        }
            
        rowSize_ = newRowSize;
        colSize_ = newColSize;
        size_ = newSize;
    }

public:
    void resize(int newRowSize, int newColSize) {
        resize(newRowSize, newColSize, true);
    }

    void resizeColumn(int newColSize){
        resize(rowSize_, newColSize);
    }

    int rowSize() const {
        return rowSize_;
    }

    /**
       \todo Make the dedicated implementation for changing the row size only
    */
    void resizeRow(int newRowSize){
        resize(newRowSize, colSize_);
    }

    int colSize() const {
        return colSize_;
    }

    void clear() {
        resize(0, 0);
    }

    const ElementType& operator()(int rowIndex, int colIndex) const {
        return buf[(offset + (rowIndex * colSize_)) % capacity_ + colIndex];
    }

    ElementType& operator()(int rowIndex, int colIndex) {
        return buf[(offset + (rowIndex * colSize_)) % capacity_ + colIndex];
    }

    const ElementType& at(int rowIndex, int colIndex) const {
        return buf[(offset + (rowIndex * colSize_)) % capacity_ + colIndex];
    }

    ElementType& at(int rowIndex, int colIndex) {
        return buf[(offset + (rowIndex * colSize_)) % capacity_ + colIndex];
    }

    Row operator[](int rowIndex) {
        return Row(*this, rowIndex);
    }

    const Row operator[](int rowIndex) const {
        return Row(*this, rowIndex);
    }
    
    Row row(int rowIndex) {
        return Row(*this, rowIndex);
    }

    const Row row(int rowIndex) const {
        return Row(*this, rowIndex);
    }
    
    Row last() {
        return Row(*this, rowSize_ - 1);
    }

    const Row last() const {
        return Row(*this, rowSize_ - 1);
    }
    
    Column column(int colIndex) {
        return Column(*this, colIndex);
    }

    const Column column(int colIndex) const {
        return Column(*this, colIndex);
    }
    
    Row append() {
        resize(rowSize_ + 1, colSize_);
        return Row(*this, rowSize_ - 1);
    }

    void pop_back() {
        resize(rowSize_ - 1, colSize_);
    }

    void pop_front(int numRows) {
        if(numRows <= 0){
            return;
        }
        if(numRows > rowSize_){
            numRows = rowSize_;
        }
        const size_t popSize = numRows * colSize_;
        ElementType* p = buf + offset;
        const ElementType* pend = buf + (offset + popSize) % capacity_;

        if(p <= pend){
            while(p != pend){
                allocator.destroy(p++);
            }
        } else {
            for(ElementType* r = buf; r != pend; ++r){
                allocator.destroy(r);
            }
            const ElementType* pterm = buf + capacity_;
            for(ElementType* r = p; r != pterm; ++r){
                allocator.destroy(r);
            }
        }
        offset = (offset + popSize) % capacity_;
        rowSize_ -= numRows;
        size_ -= popSize;
    }

    void pop_front() {
        pop_front(1);
    }

};

}

#endif
