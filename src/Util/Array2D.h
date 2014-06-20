/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_ARRAY_2D_H
#define CNOID_UTIL_ARRAY_2D_H

#include <vector>
#include <iterator>

namespace cnoid {

template <typename ElementType, typename Allocator = std::allocator<ElementType> >
class Array2D
{
    typedef Array2D<ElementType, Allocator> Array2DType;
        
public:
    typedef ElementType Element;
    typedef typename std::vector<ElementType, Allocator> Container;
    typedef typename Container::iterator iterator;

    //! \todo make const version of this class
    class Row
    {
        typename Container::iterator top;
        typename Container::iterator end_;
        int size_;
    public:

        typedef typename Container::iterator iterator;

        Row() {
            size_ = 0;
        }

        Row(const Array2D<ElementType, Allocator>& owner, int frame) {
            size_ = owner.colSize_;
            top = (const_cast<Array2D<ElementType, Allocator>&>(owner)).container.begin() + frame * size_;
            end_ = top + size_;
        }


        Row& operator=(const Row& rhs) {
            top = rhs.top;
            end_ = rhs.end_;
            size_ = rhs.size_;
            return *this;
        }
            
        bool empty() const {
            return (size_ == 0);
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

        ElementType& at(int index) {
            return top[index];
        }

        iterator begin() {
            return top;
        }

        iterator end() {
            return end_;
        }
    };

    //! \todo make const version of this class
    class Column
    {
    public:

        class iterator : public std::iterator<std::bidirectional_iterator_tag, ElementType, int> {
            typename Container::iterator current;
            int colSize;
        public:
            iterator() { }
                
            iterator(typename Container::iterator current, int colSize)
                : current(current), colSize(colSize) {
            }
            ElementType& operator*() {
                return *current;
            }
            void operator++() {
                current += colSize;
            }
            void operator--() {
                current -= colSize;
            }
            bool operator==(iterator other) const {
                return (current == other.current);
            }
            bool operator!=(iterator other) const {
                return (current != other.current);
            }
        };

    private:
        typename Container::iterator top;
        iterator end_;
        int part;
        int colSize;
        int size_;

    public:

        Column(const Array2D<ElementType, Allocator>& owner, int part) {
            top = (const_cast<Array2D<ElementType, Allocator>&>(owner)).container.begin() + part;
            this->part = part;
            colSize = owner.colSize_;
            size_ = owner.rowSize_;
            end_ =  iterator(top + colSize * size_, colSize);
        }

        bool empty() const {
            return (size_ == 0);
        }

        int size() const {
            return size_;
        }

        ElementType& operator[](int index){
            return top[index * colSize];
        }

        const ElementType& operator[](int index) const {
            return top[index * colSize];
        }

        ElementType& at(int index) {
            return top[index * colSize];
        }

        iterator begin() {
            return iterator(top, colSize);
        }
        iterator end() {
            return end_;
        }
    };

    Array2D() {
        rowSize_ = 0;
        colSize_ = 0;
    }
        
    Array2D(int rowSize, int colSize)
        : container(rowSize * colSize) {
        rowSize_ = rowSize;
        colSize_ = colSize;
    }

    Array2D(const Array2DType& org)
        : container(org.container) {
        rowSize_ = org.rowSize_;
        colSize_ = org.colSize_;
    }

    Array2DType& operator=(const Array2DType& rhs) {
        if(this != &rhs){
            container = rhs.container;
            rowSize_ = rhs.rowSize_;
            colSize_ = rhs.colSize_;
        }
        return *this;
    }
        
    virtual ~Array2D() { }

    bool empty() const {
        return container.empty();
    }

    void resize(int newRowSize, int newColSize) {
        container.resize(newRowSize * newColSize);
        rowSize_ = newRowSize;
        colSize_ = newColSize;
    }

    void clear(){
        resize(0, 0);
    }
            
    void resizeColumn(int newColSize){
        resize(rowSize_, newColSize);
    }

    int rowSize() const {
        return rowSize_;
    }

    void resizeRow(int newRowSize){
        resize(newRowSize, colSize_);
    }

    int colSize() const {
        return colSize_;
    }

    const ElementType& operator()(int rowIndex, int colIndex) const {
        return container[rowIndex * colSize_ + colIndex];
    }

    ElementType& operator()(int rowIndex, int colIndex) {
        return container[rowIndex * colSize_ + colIndex];
    }

    const ElementType& at(int rowIndex, int colIndex) const {
        return container[rowIndex * colSize_ + colIndex];
    }

    ElementType& at(int rowIndex, int colIndex) {
        return container[rowIndex * colSize_ + colIndex];
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

    Column column(int colIndex) {
        return Column(*this, colIndex);
    }

    const Column column(int colIndex) const {
        return Column(*this, colIndex);
    }

    iterator begin() {
        return container.begin();
    }

    iterator end() {
        return container.end();
    }

private:

    Container container;
    int rowSize_;
    int colSize_;
};

}

#endif
