
#ifndef CNOID_UTIL_TIMEVAL_H
#define CNOID_UTIL_TIMEVAL_H

namespace cnoid {

/**
   A class like the timeval struct used with the gettimeofday function.
   The following definition is based on the xtime class, which is
   an example presented by the documentation of the boost.chrono library.
*/
class Timeval
{
    long sec;
    long usec;

    void fixup() {
        if (usec < 0) {
            usec += 1000000;
            --sec;
        }
    }

public:

    Timeval() {
        sec = 0;
        usec = 0;
    }

    explicit Timeval(long sec, long usec) {
        sec = sec;
        usec = usec;
        if (usec < 0 || usec >= 1000000) {
            sec += usec / 1000000;
            usec %= 1000000;
            fixup();
        }
    }

    explicit Timeval(long long usec)
    {
        usec = static_cast<long>(usec % 1000000);
        sec  = static_cast<long>(usec / 1000000);
        fixup();
    }

    // explicit
    operator long long() const { return static_cast<long long>(sec) * 1000000 + usec; }

    Timeval& operator += (Timeval rhs) {
        sec += rhs.sec;
        usec += rhs.usec;
        if (usec >= 1000000) {
            usec -= 1000000;
            ++sec;
        }
        return *this;
    }

    Timeval& operator -= (Timeval rhs) {
        sec -= rhs.sec;
        usec -= rhs.usec;
        fixup();
        return *this;
    }

    Timeval& operator %= (Timeval rhs) {
        long long t = sec * 1000000 + usec;
        long long r = rhs.sec * 1000000 + rhs.usec;
        t %= r;
        sec = static_cast<long>(t / 1000000);
        usec = static_cast<long>(t % 1000000);
        fixup();
        return *this;
    }

    friend Timeval operator+(Timeval x, Timeval y) {return x += y;}
    friend Timeval operator-(Timeval x, Timeval y) {return x -= y;}
    friend Timeval operator%(Timeval x, Timeval y) {return x %= y;}

    friend bool operator==(Timeval x, Timeval y) {
        return (x.sec == y.sec && x.usec == y.usec);
    }

    friend bool operator<(Timeval x, Timeval y) {
        if (x.sec == y.sec)
            return (x.usec < y.usec);
        return (x.sec < y.sec);
    }

    friend bool operator!=(Timeval x, Timeval y) { return !(x == y); }
    friend bool operator> (Timeval x, Timeval y) { return y < x; }
    friend bool operator<=(Timeval x, Timeval y) { return !(y < x); }
    friend bool operator>=(Timeval x, Timeval y) { return !(x < y); }

    friend std::ostream& operator<<(std::ostream& os, Timeval x) {
        return os << '{' << x.sec << ',' << x.usec << '}';
    }
};

}

#endif
