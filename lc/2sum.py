'''

Given an array of integers num and an integer target, return indices
of the two numbers such that they add up to target.

You may assume that each input would have exactly one solution, and you 
may not use the same element twice.

You can return the answer in any order

'''


'''
Approach 1: Brute Force

Algorithm

The brute force approach is simple. Loop through each element x 
and find if there is another value that equals to target - x
'''

from typing import List

class Solution1:
	def twoSum(self, nums: List[int], target: int) -> List[int]:
		for i in range(len(nums)):
			for j in range(i + 1, len(nums)):
				if nums[j] == target - nums[i]:
					return [i,j]


'''
Complexity analysis:

Time complexity : O(N^2). For each element, we try to find its complement by
looping through the rest of the array which takes O(n) time. Therefore the time
complexity is O(n^2)

'''

'''
Approach 2: Two-pass hash table

Intuition

To improve the runtime complexity, we need a more efficient way to check 
if the complement exists in the array. If the complement exists, we need
to get its index. What is the best way to maintain a mapping of each element
of an array to its index? A hash table.

We can reduce the lookup time from O(n) to O(1) by trading space for speed.
A hash table is well suited for this purpose because it supports fast lookup
in 'near' constant time (barring collisions O(n) and their solutions). A hash lookup
could be amortized O(1) as long as the hash function was chosen carefully

Algorithm

A simple implementation uses two iterations. In the first iteration, we add each
element's value as a key and its index as a value to the hash table. Then, in the
second iteration, we check if the element's complement (target - nums[i]) exists
in the hash table. If it does exist, we return current element's index and its 
complement's index. Beware that the complement must not be nums[i] itself!


'''

class Solution2:
	def twoSum(self, nums: List[int], target: int) -> List[int]:
		hashmap = {}
		for i in range(len(nums)):
			hashmap[nums[i]] = i
		for i in range(len(nums)):
			complement = target - nums[i]
			if complement in hashmap and hashmap[complement] != i:
				return [i, hashmap[complement]]


'''
Approach 3: One-pass Hash Table

Algorithm

It turns out we can do it in one-pass. While we are iteratinng and inserting elements
into the hash table, we also look back to check if the current element's complement
already exists in the hash table. If it exists, we have found a solution and return 
the indices immediately.
'''

class Solution3:
	def twoSum(self, nums: List[int], target: int) -> List[int]:
		hashmap = {}
		for i in range(len(nums)):
			complement = target - nums[i]
			if complement in hashmap:
				return [i, hashmap[complement]]
			hashmap[nums[i]] = i


'''
Complexity analysis:

Time complexity: O(n). We traverse the list containing n elements only once.
Each lookup in the table costs O(1).

Space complexity: O(n). The extra space required depends on the number of items
stored in the hash table, which stores at most n elements.

'''
			
'''
Tests
'''
list1 = [1, 2, 0, 3, -1, 4, 6, -3, 1]
target = 10

solve = Solution1()
print (solve.twoSum(list1, target))

solve = Solution2()
print (solve.twoSum(list1, target))

solve = Solution3()
print (solve.twoSum(list1, target))





