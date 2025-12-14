'''
basic decorator example
'''
# define a function decorator
def my_decorator(func):
    def wrapper():
        print("some preprocessing here..")
        func()
        print("some post processing here..")

    return wrapper

# setup the decorator
@my_decorator     # commenting this just executes the function below
def say_hi():
    print("saying Hi ! ")


# in action:
say_hi()

input("enter for next:")

'''
Handling arg and return values:
To make the decorator universal, use *args and *kwargs, so it can
wrap any function regardless of its signature
'''

import functools

# here is a decorator (synctactic sugar) for debug info
# around a function

# define a generic function debug decorator
def debug(func):
    @functools.wraps(func) # a metadecorator that preserves metadata like func.__name__

    def wrapper(*args, **kwargs):
        print(f"Calling {func.__name__} with {args} and {kwargs}")
        result = func(*args, *kwargs)
        print(f"Function {func.__name__} returned {result}")
        return result

    return wrapper

# setup the decorator
@debug
def add_numbers(a, b):
    return a+b

# in action
add_numbers(100.3, 10.2)

# note: @functools.wraps: Essential for maintaining the original function's name and docstring

input("enter for next:")

'''
Can pass arguements to a decorator, to further manipulate the function
it decorates
'''

# set up a repeater decorator

def repeat(num_times):

    def decorator_repeat(func):
        @functools.wraps(func)

        def wrapper(*args, **kwargs):
            for _ in range(num_times):
                value = func(*args, **kwargs) # input: can manipulate more
            return value # output: can manipulate more
        return wrapper
    return decorator_repeat

@repeat(num_times=5)
def greet(name):
    print(f"Hello {name}")

greet("Anupam")

'''
Real-World Use Cases:

Decorators are extensively used in modern Python frameworks for cross-cutting concerns. 

Timing Execution: Used for performance profiling.
Authentication/Authorization: Checking user permissions before running a web view.
Caching (Memoization): Storing results of expensive computations.
Web Routing: Frameworks like Flask use @app.route("/") to map URLs to functions. 

'''

input("enter for next:")

'''
Class based decorators
1. Can use class as a decorator by implementing the __call__ method 
2. Rather than decorating a method inside a class, can decorate the entire class to 
   modify its attributes and behavior upon definition

Both (1) and (2) are shown below
'''

# showing 1..

class CountCalls:

    # decorator definition
    def __init__(self, func):
        self.func = func
        self.num_calls = 0

    def __call__(self, *args, **kwargs):
        self.num_calls += 1
        print(f"Call {self.num_calls} of {self.func.__name__}")
        return self.func(*args, **kwargs)

@CountCalls
def simple_task():
    print("Task complete.")

simple_task()
simple_task()
simple_task()

input("enter for next:")

# showing 2..

def add_info(cls):
    cls.extra_info = "Decorated Class"
    return cls

@add_info
class MyClass:
    pass

print(MyClass.extra_info)  # Output: Decorated Class



