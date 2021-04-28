#ifndef _QUEUE_H_
#define _QUEUE_H_

// A class to store a queue
class Queue
{
    int *arr;     // array to store queue elements
    int capacity; // maximum capacity of the queue
    int front;    // front points to the front element in the queue (if any)
    int rear;     // rear points to the last element in the queue

public:
    Queue(int size); // constructor
    ~Queue();        // destructor

    void dequeue();
    void enqueue(int x);
    int head3();
    int tail3();
};

#endif // _QUEUE_H_
