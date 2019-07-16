/*
  The classes implemented in this file are based on the boost.tokenizer implementation.
  The copyright of the original implementation is as follows:
   (c) Copyright Jeremy Siek and John R. Bandela 2001. 
   Distributed under the Boost Software License, Version 1.0.
   (See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CNOID_UTIL_TOKENIZER_H
#define CNOID_UTIL_TOKENIZER_H

#include <string>
#include <algorithm>
#include <stdexcept>
#include <cctype>
#include <cassert>

namespace cnoid {

struct escaped_list_error : public std::runtime_error {
    escaped_list_error(const std::string& what_arg) : std::runtime_error(what_arg) { }
};


template<class Char, class Traits = typename std::basic_string<Char>::traits_type>
class EscapedListSeparator
{
private:
    typedef std::basic_string<Char,Traits> string_type;
    struct char_eq {
        Char e_;
        char_eq(Char e):e_(e) { }
        bool operator()(Char c) {
            return Traits::eq(e_,c);
        }
    };
    string_type  escape_;
    string_type  c_;
    string_type  quote_;
    bool last_;

    bool is_escape(Char e){
        char_eq f(e);
        return std::find_if(escape_.begin(),escape_.end(),f)!=escape_.end();
    }
    
    bool is_c(Char e){
        char_eq f(e);
        return std::find_if(c_.begin(),c_.end(),f)!=c_.end();
    }
    
    bool is_quote(Char e){
        char_eq f(e);
        return std::find_if(quote_.begin(),quote_.end(),f)!=quote_.end();
    }
    
    template <typename iterator, typename Token>
    void do_escape(iterator& next,iterator end,Token& tok){
        if(++next == end){
            throw escaped_list_error("cannot end with escape");
        }
        if(Traits::eq(*next,'n')){
            tok += '\n';
            return;
        } else if(is_quote(*next)){
            tok += *next;
            return;
        } else if(is_c(*next)){
             tok += *next;
             return;
        } else if (is_escape(*next)){
            tok += *next;
            return;
        } else {
            throw escaped_list_error("unknown escape sequence");
        }
    }

public:
    explicit EscapedListSeparator(Char  e = '\\', Char c = ',',Char  q = '\"')
        : escape_(1,e), c_(1,c), quote_(1,q), last_(false) { }

    EscapedListSeparator(string_type e, string_type c, string_type q)
        : escape_(e), c_(c), quote_(q), last_(false) { }

    void reset(){ last_=false; }

    template <typename InputIterator, typename Token>
    bool operator()(InputIterator& next,InputIterator end,Token& tok) {
        bool bInQuote = false;
        tok = Token();

        if(next == end){
            if(last_){
                last_ = false;
                return true;
            } else {
                return false;
            }
        }
        last_ = false;
        for(; next != end; ++next){
            if(is_escape(*next)){
                do_escape(next, end, tok);
            } else if(is_c(*next)){
                if(!bInQuote){
                    // If we are not in quote, then we are done
                    ++next;
                    // The last character was a c, that means there is
                    // 1 more blank field
                    last_ = true;
                    return true;
                }
                else tok += *next;
            }
            else if (is_quote(*next)){
                bInQuote=!bInQuote;
            } else {
                tok += *next;
            }
        }
        return true;
    }
};


enum EmptyTokenPolicy { DROP_EMPTY_TOKENS, KEEP_EMPTY_TOKENS };

template <typename Char, typename Tr = typename std::basic_string<Char>::traits_type>
class CharSeparator
{
    typedef std::basic_string<Char,Tr> string_type;

public:
    explicit
    CharSeparator(const Char* dropped_delims,
                  const Char* kept_delims = 0,
                  EmptyTokenPolicy empty_tokens = DROP_EMPTY_TOKENS)
        : m_dropped_delims(dropped_delims),
          m_use_ispunct(false),
          m_use_isspace(false),
          m_empty_tokens(empty_tokens),
          m_output_done(false)
    {
        // Borland workaround
        if (kept_delims){
            m_kept_delims = kept_delims;
        }
    }

    // use ispunct() for kept delimiters and isspace for dropped.
    explicit
    CharSeparator()
        : m_use_ispunct(true),
          m_use_isspace(true),
          m_empty_tokens(DROP_EMPTY_TOKENS),
          m_output_done(false) { }
    
    void reset() { }

    template <typename InputIterator, typename Token>
    bool operator()(InputIterator& next, InputIterator end, Token& tok)
    {
        // skip past all dropped_delims
        if(m_empty_tokens == DROP_EMPTY_TOKENS){
            for(; next != end && is_dropped(*next); ++next){ }
        }

        InputIterator start(next);

        if(m_empty_tokens == DROP_EMPTY_TOKENS){
            if(next == end){
                return false;
            }
            // if we are on a kept_delims move past it and stop
            if(is_kept(*next)){
                ++next;
            } else {
                // append all the non delim characters
                for(; next != end && !is_dropped(*next) && !is_kept(*next); ++next);
            }
        } else { // m_empty_tokens == keep_empty_tokens

            // Handle empty token at the end
            if(next == end){
                if(m_output_done == false){
                    m_output_done = true;
                    tok.assign(start, next);
                    return true;
                } else {
                    return false;
                }
            }

            if(is_kept(*next)){
                if(m_output_done == false){
                    m_output_done = true;
                } else {
                    ++next;
                    m_output_done = false;
                }
            } else if(m_output_done == false && is_dropped(*next)){
                m_output_done = true;
            } else {
                if(is_dropped(*next)){
                    start=++next;
                }
                for(; next != end && !is_dropped(*next) && !is_kept(*next); ++next);
                m_output_done = true;
            }
        }
        tok.assign(start, next);
        return true;
    }

private:
    string_type m_kept_delims;
    string_type m_dropped_delims;
    bool m_use_ispunct;
    bool m_use_isspace;
    EmptyTokenPolicy m_empty_tokens;
    bool m_output_done;

    bool is_kept(Char E) const
    {
        if(m_kept_delims.length()){
            return m_kept_delims.find(E) != string_type::npos;
        } else if(m_use_ispunct){
            return std::ispunct(E) != 0;
        } else {
            return false;
        }
    }
    bool is_dropped(Char E) const
    {
        if(m_dropped_delims.length()){
            return m_dropped_delims.find(E) != string_type::npos;
        } else if(m_use_isspace){
            return std::isspace(E) != 0;
        } else {
            return false;
        }
    }
};


template <class TokenizerFunc, class Iterator, class Type>
class TokenIterator
{
    friend class iterator_core_access; 

    TokenizerFunc f_;
    Iterator begin_;
    Iterator end_;
    bool valid_;
    Type tok_;

    void increment(){
        assert(valid_);
        valid_ = f_(begin_,end_,tok_);
    }

    const Type& dereference() const {
        assert(valid_);
        return tok_;
    }
    
    template<class Other>
    bool equal(const Other& a) const {
        return (a.valid_ && valid_)
            ?( (a.begin_==begin_) && (a.end_ == end_) )
            :(a.valid_==valid_);

    }

    void initialize(){
        if(valid_) return;
        f_.reset();
        valid_ = (begin_ != end_)?
            f_(begin_,end_,tok_):false;
    }
    
public:
    TokenIterator():begin_(),end_(),valid_(false),tok_() { }

    TokenIterator(TokenizerFunc f, Iterator begin, Iterator e = Iterator())
        : f_(f),begin_(begin),end_(e),valid_(false),tok_(){ initialize(); }

    TokenIterator(Iterator begin, Iterator e = Iterator())
        : f_(),begin_(begin),end_(e),valid_(false),tok_() {initialize();}

    TokenIterator& operator++(){
        increment();
        return *this;
    }
     
    TokenIterator operator++(int){
        TokenIterator retval = *this;
        increment();
        return retval;
    }
    
    bool operator==(const TokenIterator& other) const {
        return equal(other);
    }

    bool operator!=(const TokenIterator& other) const {
        return !(*this == other);
    }
    
    const Type& operator*() const {
        return dereference();
    }

    Type const * operator->() const {
        return &dereference();
    }
    
    Iterator base()const{return begin_;}

    Iterator end()const{return end_;}

    TokenizerFunc tokenizer_function()const{return f_;}

    Type current_token()const{return tok_;}

    bool at_end()const{return !valid_;}
};


template <
    class TokenizerFunc = CharSeparator<char>,
    class Iterator = std::string::const_iterator,
    class Type = std::string
    >
class TokenIteratorGenerator
{
public:
    typedef TokenIterator<TokenizerFunc,Iterator,Type> type;
};


// Type has to be first because it needs to be explicitly specified
// because there is no way the function can deduce it.
template<class Type, class Iterator, class TokenizerFunc>
typename TokenIteratorGenerator<TokenizerFunc,Iterator,Type>::type
make_token_iterator(Iterator begin, Iterator end,const TokenizerFunc& fun){
    typedef typename
        TokenIteratorGenerator<TokenizerFunc,Iterator,Type>::type ret_type;
    return ret_type(fun,begin,end);
}


template <
    typename TokenizerFunc = CharSeparator<char>, 
    typename Iterator = std::string::const_iterator,
    typename Type = std::string
    >
class Tokenizer
{
private:
    typedef TokenIteratorGenerator<TokenizerFunc,Iterator,Type> TGen;
    typedef typename TGen::type iter;
    
public:
    typedef iter iterator;
    typedef iter const_iterator;
    typedef Type value_type;
    typedef value_type& reference;
    typedef const value_type& const_reference;
    typedef value_type* pointer;
    typedef const pointer const_pointer;
    typedef void size_type;
    typedef void difference_type;

    Tokenizer(const TokenizerFunc& f)
        : f_(f) { }

    Tokenizer(Iterator first, Iterator last, const TokenizerFunc& f = TokenizerFunc()) 
        : first_(first), last_(last), f_(f) { }
        
    template <typename Container>
    Tokenizer(const Container& c)
        : first_(c.begin()), last_(c.end()), f_() { }
    
    template <typename Container>
    Tokenizer(const Container& c,const TokenizerFunc& f)
        : first_(c.begin()), last_(c.end()), f_(f) { }
    
    void assign(Iterator first, Iterator last){
        first_ = first;
        last_ = last;
    }
    
    void assign(Iterator first, Iterator last, const TokenizerFunc& f){
        assign(first,last);
        f_ = f;
    }
    
    template <typename Container>
    void assign(const Container& c){
        assign(c.begin(),c.end());
    }
    
    template <typename Container>
    void assign(const Container& c, const TokenizerFunc& f){
        assign(c.begin(),c.end(),f);
    }
    
    iter begin() const { return iter(f_,first_,last_); }
    iter end() const { return iter(f_,last_,last_); }
    
private:
    Iterator first_;
    Iterator last_;
    TokenizerFunc f_;
};

}

#endif
