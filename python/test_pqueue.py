# Demo of using PQueue class

from pqueue import PQueue




def main():
    q = PQueue()
    q.enqueue("is", 5)
    q.enqueue("computer", 3)
    q.enqueue("rhodes", 1)
    q.enqueue("awesome", 6)
    q.enqueue("college", 2)
    q.enqueue("science", 4)

    while not q.empty():
        s = q.dequeue()
        print("dequeued:", s)

main()
