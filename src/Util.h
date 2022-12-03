#ifndef INCL_UTIL_H
#define INCL_UTIL_H

#include <Arduino.h>
#include <functional>
#include <FS.h>

class Util {
public:
    static void durationToStr(String *val, int64_t millis);

    struct ThreadOptions {
        uint32_t stackSize = 0;
        unsigned priority = 0; // 0 means caller task's priority
    };
    static void runAsThread(const char *taskName, std::function<void ()> fn);
    static void runAsThread(const char *taskName, ThreadOptions &opt, std::function<void ()> fn);
    static void base64Encode(String *result, char *buf, size_t size);
};

#if 0
template <class T>
class VectorRing {
private:
    std::vector<T> data;
    int head;
    int tail;
public:
    void push_back(T val);
    T pop_front();
};

inline template <class T> VectorRing::VectorRing()
{
    head = 0;
    tail = 0;
}

inline template <class T> VectorRing::push_back(int i)
{
    if (head + 1 )
}


class StringRing {
private:
    size_t maxLen;
    std::vector<char *> lines;
    int linesHead;
    int linesTail;
    char *buf;
    size_t bufSize;
    char *head;
    char *tail;
    uint32_t tailEntryId;
    uint32_t headEntryId;
public:
    void setBufferSize(size_t size);
    // maximum length of a string (it's truncated if longer)
    void setMaxLen(size_t len);

    uint32_t push(const char *str);
    char *get(uint32_t);
    uint32_t getMin();
    uint32_t getMax();
};
#endif

/*****************************\
 *
 * There are 2 possible markers: 0x1E and 0x1F. They must not be present in user data.
 *
 * Every time we write a record, we end by writing marker A then marker B, and reposition
 * on top of maker B (so it is overwritten on next write). We may decide to wrap around:
 * we exchange markers A and B, start from beginning, and write marker A, marker B and
 * reposition on top of marker B.
 *
 * Note: if we crash after having written a byte (overwriting a marker B) and before
 * having written the last marker A and marker B, we may no longer have a marking for
 * the last written position... and the file wouldn't be consistent. TODO. VERIFY on LittleFS.
 * LittleFS: "Additionally, file updates are not actually committed to the filesystem
 * until sync or close is called on the file." -- So should be OK.
 *
 * When we open the file, we must find the position where we left off. For this,
 * first we scan the file from the beginning. The first marker we encounter (out
 * of the two possible markers) is marker A (and the other marker is marker B). We
 * keep reading until we find the first position of marker B (or EOF). Now
 * we know the position we left off is just at the first marker B.
 *
 * To read the file, we start from the first marker B, skip one recod (it may well
 * be partially overwritten) and start reading from there, until we find the first
 * marker A. At that point we ignore the rest and start from the beginning of
 * the file, reading records limited by marker A, until we find a first marker B.
 *
 ******************************/
class RotatingFile : public Print { // this is for writing only
    File pFile;
    size_t sizeLimit;
    int markerA;
    int markerB;
    uint8_t markerAB[2]; // used to pFile.write() both markers at once
    int lastMarkerAPos;
    int firstMarkerBPos;
public:
    RotatingFile();
    void openForWrite(const char *fileName, FS *fs, size_t sizeLimit);
    virtual ~RotatingFile();
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *buffer, size_t size);
    size_t write(const char *buffer, size_t size) { return write((const char *)buffer, size); };
    void terminateRecord();
};


// With accompanying data

template <class T, class Other>
class CallbackList2 {
    struct Entry {
        std::function<T> t;
        int id;
        Other other;
    };
    std::vector<Entry> callbackList;
    int nextId;
public:
    int add(std::function<T> t, Other other);
    void remove(int id);
    int size();
    /** returns the index where 'id' is found, or -1 if not found */
    int find(int id);
    std::function<T> get(int i);
    Other getOther(int i);
};

template <class T, class Other>
inline int CallbackList2<T, Other>::add(std::function<T> t, Other other) {
    Entry e;
    e.t = t;
    e.id = nextId;
    e.other = other;
    callbackList.push_back(e);
    ++nextId;
    return e.id;
}

template <class T, class Other>
inline int CallbackList2<T, Other>::find(int id) {
    for (auto p = callbackList.begin(); p != callbackList.end(); p++) {
        if (p->id == id) {
            return p - callbackList.begin();
        }
    }
    return -1;
}

template <class T, class Other>
inline void CallbackList2<T, Other>::remove(int id) {
    for (auto p = callbackList.begin(); p != callbackList.end(); p++) {
        if (p->id == id) {
            callbackList.erase(p);
            return;
        }
    }
}

template <class T, class Other>
inline int CallbackList2<T, Other>::size() {
    return callbackList.size();
}

template <class T, class Other>
inline std::function<T> CallbackList2<T, Other>::get(int i) {
    return callbackList[i].t;
}

template <class T, class Other>
inline Other CallbackList2<T, Other>::getOther(int i) {
    return callbackList[i].other;
}


// No accompanying data

template <class T>
class CallbackList {
    struct Entry {
        std::function<T> t;
        int id;
    };
    std::vector<Entry> callbackList;
    int nextId;
public:
    int add(std::function<T> t);
    void remove(int id);
    int size();
    /** returns the index where 'id' is found, or -1 if not found */
    int find(int id);
    std::function<T> get(int i);
    void clear();
};

template <class T>
inline int CallbackList<T>::add(std::function<T> t) {
    Entry e;
    e.t = t;
    e.id = nextId;
    callbackList.push_back(e);
    ++nextId;
    return e.id;
}

template <class T>
inline int CallbackList<T>::find(int id) {
    for (auto p = callbackList.begin(); p != callbackList.end(); p++) {
        if (p->id == id) {
            return p - callbackList.begin();
        }
    }
    return -1;
}

template <class T>
inline void CallbackList<T>::remove(int id) {
    for (auto p = callbackList.begin(); p != callbackList.end(); p++) {
        if (p->id == id) {
            callbackList.erase(p);
            return;
        }
    }
}

template <class T>
inline int CallbackList<T>::size() {
    return callbackList.size();
}

template <class T>
inline std::function<T> CallbackList<T>::get(int i) {
    return callbackList[i].t;
}

template <class T>
inline void CallbackList<T>::clear() {
    callbackList.clear();
}

// Register some data and return an ID, obtain the data given it's ID,
// remove data that's registered with a given ID.
template <class T>
class Register {
    struct Entry {
        T t;
        int id;
    };
    std::vector<Entry> list;
    int nextId;
public:
    int add(T t);
    void remove(int id);
    int size();
    // The returned pointer is not valid if this is modified in any way (add(), remove()...)
    T *find(int id);
    class It {
        int pos;
        It();
    public:
        bool hasNext();
        T &next();
        void remove();
    };
    It iterator();
};

template <class T>
inline int Register<T>::add(T t) {
    Entry e;
    e.t = t;
    e.id = nextId;
    list.push_back(e);
    ++nextId;
    return e.id;
}

template <class T>
inline T *Register<T>::find(int id) {
    for (auto p = list.begin(); p != list.end(); p++) {
        if (p->id == id) {
            return &p->t;
        }
    }
    return nullptr;
}

template <class T>
inline void Register<T>::remove(int id) {
    for (auto p = list.begin(); p != list.end(); p++) {
        if (p->id == id) {
            list.erase(p);
            return;
        }
    }
}

template <class T>
inline int Register<T>::size() {
    return list.size();
}

template <class T> Register<T>::It::It()
: pos(0)
{}

template <class T> bool Register<T>::It::hasNext()
{
    return pos < list.size();
}

template <class T> T &Register<T>::It::next()
{
    ++pos;
    return list[pos];
}

template <class T> void Register<T>::It::remove()
{
    list.erase(pos);
}



#endif
