#!/usr/bin/env python
 
from __future__ import print_function
 
from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy
 
def handle_add_two_ints(req):
     print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
     return AddTwoIntsResponse(req.a + req.b)
 
def add_two_ints_server():
     rospy.init_node('add_two_ints_server')
     s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
     #This declares a new service named add_two_ints with the AddTwoInts service type. 
     # All requests are passed to handle_add_two_ints function. 
     # handle_add_two_ints is called with instances of AddTwoIntsRequest and returns instances of AddTwoIntsResponse.
     print("Ready to add two ints.")
     rospy.spin()
 
if __name__ == "__main__":
     add_two_ints_server()