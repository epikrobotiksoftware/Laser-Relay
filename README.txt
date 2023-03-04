This package takes two laser scan topics and relays them into one output topic.
The reading sources are not changed and the read values are not calculated again.
Note: Make sure both your laser sources are publising at the same rate for more stability.

Usage:
- Declare the topic_in1, topic_in2, topic_out arguments at the top.
- Include the laser_relay.launch.py file into your launch file.
- Run

Refer to test.launch.py for a example usage.