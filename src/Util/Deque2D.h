/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_DEQUE_2D_H
#define CNOID_UTIL_DEQUE_2D_H

#include <memory>
#include <iterator>

namespace cnoid {

template <typename ElementType, typename Allocator = std::allocator<ElementType>>
class Deque2D
{
    typedef Deque2D<ElementType, Allocator> Deque2DType;
    
public:
    typedef ElementType value_type;
    typedef ElementType Element; ///< \deprecated. Use value_type.
    
    class const_iterator : public std::iterator<std::random_access_iterator_tag, ElementType> {

        friend class Deque2D<ElementType, Allocator>;
        
    protected:
        ElementType* current;
        ElementType* term;
        ElementType* buf;

        const_iterator(const Deque2DType& owner, ElementType* pos) {
            current = pos;
            buf = owner.buf;
            term = buf + owner.capacity_;
        }
            
    public:
        const_iterator() { }

        const_iterator(const const_iterator& org) {
            current = org.current;
            term = org.term;
            buf = org.buf;
        }
        
        const Element& operator*() const {
            return *current;
        }
        const_iterator& operator++() {
            ++current;
            if(current == term){
                current = buf;
            }
            return *this;
        }
        const_iterator& operator--() {
            if(current == buf){
                current = term - 1;
            } else {
                --current;
            }
        }
        const_iterator& operator+=(size_t n){
            current += n;
            if(current >= term){
                current = buf + (current - term);
            }
            return *this;
        }
        const_iterator& operator-=(size_t n){
            current -= n;
            if(current < buf){
                current = term - (buf - current);
            }
            return *this;
        }
        const_iterator operator+(size_t n){
            const_iterator iter(*this);
            iter += n;
            return iter;
        }
        const_iterator operator-(size_t n){
            const_iterator iter(*this);
            iter -= n;
            return iter;
        }
        bool operator==(const const_iterator& rhs) const {
            return (current == rhs.current);
        }
        bool operator!=(const const_iterator& rhs) const {
            return (current != rhs.current);
        }
        bool operator<(const const_iterator& rhs) const{
            return current < rhs.current;
        }
    };

    class iterator : public const_iterator {

        friend class Deque2D<ElementType, Allocator>;

        iterator(Deque2DType& owner, ElementType* pos) : const_iterator(owner, pos) { }

    public:
        iterator() { }
        iterator(const iterator& org) : const_iterator(org) { }
        
        Element& operator*() {
            return *const_iterator::current;
        }
        iterator& operator+=(size_t n){
            const_iterator::current += n;
            if(const_iterator::current >= const_iterator::term){
                const_iterator::current = const_iterator::buf + (const_iterator::current - const_iterator::term);
            }
            return *this;
        }
        iterator& operator-=(size_t n){
            const_iterator::current -= n;
            if(const_iterator::current < const_iterator::buf){
                const_iterator::current = const_iterator::term - (const_iterator::buf - const_iterator::current);
            }
            return *this;
        }
        iterator operator+(size_t n){
            iterator iter(*this);
            iter += n;
            return iter;
        }
        iterator operator-(size_t n){
            iterator iter(*this);
            iter -= n;
            return iter;
        }
    };
        
    iterator begin() {
        return iterator(*this, buf + offset);
    }

    const_iterator cbegin() const {
        return const_iterator(*this, buf + offset);
    }

    iterator end() {
        return end_;
    }

    const_iterator cend() const {
        return end_;
    }

    class Row
    {
        ElementType* top;
        int size_;

    public:
        Row() {
            size_ = 0;
        }
        
        Row(const Deque2D<ElementType, Allocator>& owner, int rowIndex) {
            size_ = owner.colSize_;
            top = owner.buf;
            if(owner.capacity_ > 0){
                top += (owner.offset + rowIndex * owner.colSize_) % owner.capacity_;
            }
        }

        bool empty() const {
            return (size_ == 0);
        }

        int size() const {
            return size_;
        }

        Element& operator[](int index){
            return top[index];
        }

        const Element& operator[](int index) const {
            return top[index];
        }

        Element& at(int index) {
            return top[index];
        }

        Element* begin() {
            return top;
        }

        Element* end() {
            return top + size_;
        }
    };

    class Column
    {
    public:
        Column() {
            colSize = 0;
            rowSize = 0;
        }
        
        Column(const Deque2D<ElementType, Allocator>& owner, int column) {
            top = owner.buf + column;
            offset = owner.offset;
            colSize = owner.colSize_;
            capacity = owner.capacity_;
            rowSize = owner.rowSize_;
            end_ = iterator(*this, top + ((owner.capacity_ > 0) ? ((offset + owner.size_) % owner.capacity_) : 0));
        }

        bool empty() const {
            return (rowSize == 0);
        }

        int size() const {
            return rowSize;
        }

        Element& operator[](int rowIndex){
            return top[(offset + rowIndex * colSize) % capacity];
        }

        const Element& operator[](int rowIndex) const {
            return top[(offset + rowIndex * colSize) % capacity];
        }

        Element& at(int index) {
            return top[index * colSize];
        }

        class iterator : public std::iterator<std::bidirectional_iterator_tag, ElementType, int> {

            ElementType* current;
            ElementType* term;
            ElementType* top;
            int colSize;
                
        public:
                
            iterator() { }
                
            iterator(Column& column, Element* pos){
                current = pos;
                top = column.top;
                term = top + column.capacity;
                colSize = column.colSize;
            }
            Element& operator*() {
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
        ElementType* top;
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

        resizeMain(rowSize, colSize, false);
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
        end_ = iterator(*this, buf + ((capacity_ > 0) ? ((offset + size_) % capacity_) : 0));
    }

    Deque2DType& operator=(const Deque2DType& rhs) {
        if(this != &rhs){
            resizeMain(rhs.rowSize_, rhs.colSize_, false);
            iterator p = begin();
            const_iterator q = rhs.cbegin();
            const_iterator qend = rhs.cend();
            while(q != qend){
                *p = *q;
                ++p;
                ++q;
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

        ElementType* newBuf;
        if(newCapacity > 0){
            newBuf = allocator.allocate(newCapacity);
        } else {
            newBuf = 0;
        }
        ElementType* p = newBuf;
        ElementType* pend = newBuf + newSize;

        if(capacity_ > 0){
            ElementType* qend = buf + (offset + size_) % capacity_;
            
            // copy the existing elements
            if(newCapacity > 0 && newColSize == colSize_ && doCopy){
                ElementType* q = buf + offset;
                if(q <= qend){
                    while(q != qend && p != pend){
                        allocator.construct(p++, *q++);
                    }
                } else {
                    for(ElementType* r = buf; r != qend && p != pend; ++r){
                        allocator.construct(p++, *r);
                    }
                    ElementType* qterm = buf + capacity_;
                    for(ElementType* r = q; r != qterm && p != pend; ++r){
                        allocator.construct(p++, *r);
                    }
                }
            }
            // destory the old elements
            ElementType* q = buf + offset;
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

        if(buf){
            allocator.deallocate(buf, capacity_);
        }
        buf = newBuf;
        capacity_ = newCapacity;
        offset = 0;
    }

    void resizeMain(int newRowSize, int newColSize, bool doCopy) {

        const int newSize = newRowSize * newColSize;

        if(newSize == 0){
            reallocMemory(newColSize, newSize, 0, false);
            
        } else {
            // The area for the 'end' iterator should be reserved
            const int minCapacity = newSize + newColSize;
        
            if(capacity_ > 0 && minCapacity <= capacity_){
                if(newColSize != colSize_ && (capacity_ % newColSize > 0)){
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
                    if(capacity_ > 0){
                        buf = allocator.allocate(minCapacity);
                        ElementType* p = buf;
                        ElementType* pend = buf + newSize;
                        // construct new elements
                        while(p != pend){
                            allocator.construct(p++, ElementType());
                        }
                    }
                } else {
                    int newCapacity;
                    const int expandedSize = size_ * 3 / 2;
                    if(expandedSize > newSize){
                        newCapacity = expandedSize - (expandedSize % newColSize) + newColSize;
                    } else {
                        newCapacity = minCapacity;
                    }
                    reallocMemory(newColSize, newSize, newCapacity, doCopy);
                }
            }
        }
            
        rowSize_ = newRowSize;
        colSize_ = newColSize;
        size_ = newSize;
        end_ = iterator(*this, buf + ((capacity_ > 0) ? ((offset + size_) % capacity_) : 0));
    }

public:
    void resize(int newRowSize, int newColSize) {
        resizeMain(newRowSize, newColSize, true);
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

    const Element& operator()(int rowIndex, int colIndex) const {
        return buf[(offset + (rowIndex * colSize_)) % capacity_ + colIndex];
    }

    Element& operator()(int rowIndex, int colIndex) {
        return buf[(offset + (rowIndex * colSize_)) % capacity_ + colIndex];
    }

    const Element& at(int rowIndex, int colIndex) const {
        return buf[(offset + (rowIndex * colSize_)) % capacity_ + colIndex];
    }

    Element& at(int rowIndex, int colIndex) {
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

    void pop_front(int numRows = 1) {
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

private:
    Allocator allocator;
    ElementType* buf;
    int offset;
    int rowSize_;
    int colSize_;
    int capacity_;
    int size_;
    iterator end_;
};

}

#endif
