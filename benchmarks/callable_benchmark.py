#type: ignore
import numpy as np
import time

class multiplier:
    def method_call_product(self, params):
        a,b = params
        return a*b
    
    def callable_product(self):
        def prod(params):
            a,b = params
            return a*b
        return prod
    
def product(params):
    a,b = params
    return a*b

def as_perc(datum, value):
    return 100*(value/datum)

count = 10**7

rand_inputs = np.random.rand(count, 2)

#method call
m = multiplier()
start_time = time.time()
for input in rand_inputs:
    output = m.method_call_product(input)
end_time = time.time()
time_method = end_time-start_time
print("method_call: " + str(time_method))


#func call from method
start_time = time.time()
for input in rand_inputs:
    output = m.callable_product()(input)
end_time = time.time()
time_func_from_method = end_time-start_time
print("func_call_from_method: " + str(time_func_from_method))


#func call without method
prod = m.callable_product()
start_time = time.time()
for input in rand_inputs:
    output = prod(input)
end_time = time.time()
time_func = end_time-start_time
print("func_call: " + str(time_func))

#pure function call
start_time = time.time()
for input in rand_inputs:
    output = product(input)
end_time = time.time()
time_func_pure = end_time-start_time
print("func_call: " + str(time_func_pure))



print("as percent:")
print(as_perc(time_method, time_method))
print(as_perc(time_method, time_func_from_method))
print(as_perc(time_method, time_func))
print(as_perc(time_method, time_func_pure))