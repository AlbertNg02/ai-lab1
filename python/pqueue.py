# Simple priority queue class; sorts by minimum priority.

from dataclasses import dataclass, field
from typing import Any
from queue import PriorityQueue

@dataclass(order=True)
class PrioritizedItem:
	priority: int
	item: Any=field(compare=False)

class PQueue:
	def __init__(self):
		self.the_queue = PriorityQueue()
		
	def empty(self):
		return self.the_queue.empty()
		
	def enqueue(self, item, priority):
		self.the_queue.put(PrioritizedItem(priority, item))

	def dequeue(self):
		return self.the_queue.get().item
		
	def size(self):
		return self.the_queue.qsize()
		
	def debug_print(self):
		print(self.the_queue)
