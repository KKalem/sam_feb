# sam_feb

Clone bee_tea from here:
```
https://github.com/KKalem/bee_tea
```
then catkin_make as usual.
If some errors pop up about message generation and stuff, try catkin_make_isolated.
Or try freshly installing the whole thing, just make sure the old "Bea_Tea" stuff is gone somehow.


With roscore and all the SAM stuff running:

Terminal 1:
```
rosrun bee_tea bt_example.py
```
This will run a very simple behaviour tree that will run an action named "test" once and wait for it to complete.
Use this to test your actions individually.


Then you can run on terminal 2 one of:
```
rosrun sam_feb sam_sine_action.py
rosrun sam_feb sam_emergency.py
```

Then modify these to your hearts content so that they actually do something useful.
Example publisher/subscribers are in sam_sine_action.py.



