#include "Queue.h"

// Constructor to initialize a queue
Queue::Queue(int size)
{
    arr = new int[size];
    for (int i = 0; i < size; i++)
        arr[i] = 500; // prevent false start
    capacity = size;
    front = 0;
    rear = size - 1;
}

// Destructor to free memory allocated to the queue
Queue::~Queue()
{
    delete[] arr;
}

// Utility function to dequeue the front element
void Queue::dequeue()
{
    front = (front + 1) % capacity;
}

// Utility function to add an item to the queue
void Queue::enqueue(int item)
{
    dequeue();
    rear = (rear + 1) % capacity;
    arr[rear] = item;
}

int Queue::head3()
{
    return arr[front] + arr[(front + 1) % capacity] + arr[(front + 2) % capacity];
}

int Queue::tail3()
{

    return arr[rear] + arr[(rear + capacity - 1) % capacity] + arr[(rear + capacity - 2) % capacity];
}
