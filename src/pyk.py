#!/usr/bin/env python
import rospy
import pygame
import gazebo_msgs.msg
import hector_uav_msgs.msg

total_points = 0
prev_count = 2
first_taken = False
end_1 = (0, 0)
pygame.init()
S = pygame.display.set_mode((700, 700))

def handle_model_states(msg):
	global prev_count, S
	try:
		n = len(msg.name)
		if n > prev_count:
			r_pix_x = int(350 + msg.pose[1].position.x * 35)
			r_pix_y = int(350 - msg.pose[1].position.y * 35)
			pygame.draw.circle(pygame.display.get_surface(), (255, 0, 0), (r_pix_x, r_pix_y), 3, 0)
			for i in range(2, n):
				o_pix_x = int(350 + msg.pose[i].position.x * 35)
				o_pix_y = int(350 - msg.pose[i].position.y * 35)
				pygame.draw.circle(pygame.display.get_surface(), (0, 0, 0, 100), (o_pix_x, o_pix_y), 34, 0)
			prev_count = n
	except IndexError as e:
		# raise e
		return

def handle_sampled_point(msg):
	global first_taken, end_1, S, total_points
	p_pix_x = int(350 + msg.x * 35)
	p_pix_y = int(350 - msg.y * 35)
	if not first_taken:
		if msg.z < 0:
			pygame.draw.circle(pygame.display.get_surface(), (133, 0, 113), (p_pix_x, p_pix_y), 3, 0)
			first_taken = False
		elif msg.z == 0:
			pygame.draw.circle(pygame.display.get_surface(), (0, 0, 255), (p_pix_x, p_pix_y), 3, 0)
			end_1 = (p_pix_x, p_pix_y)
			first_taken = True
	else:
		if msg.z < 0:
			pygame.draw.circle(pygame.display.get_surface(), (133, 0, 113), (p_pix_x, p_pix_y), 3, 0)
		elif 0 < msg.z < 100:
			pygame.draw.circle(pygame.display.get_surface(), (0, 0, 255), (p_pix_x, p_pix_y), 3, 0)
			pygame.draw.line(pygame.display.get_surface(), (235, 231, 0), end_1, (p_pix_x, p_pix_y), 1)
		elif msg.z > 100:
			pygame.draw.line(pygame.display.get_surface(), (235, 231, 0), end_1, (p_pix_x, p_pix_y), 1)
		first_taken = False
	total_points += 1
	pygame.display.update()

def handle_sampled_point2(msg):
	global S
	p_pix_x = int(350 + msg.x * 35)
	p_pix_y = int(350 - msg.y * 35)
	color_tuple = (0, 0, 255)
	rospy.loginfo('Received point: (%f, %f)' % (msg.x, msg.y))
	if msg.z > 0:
		color_tuple = (0, 255, 0)
	elif msg.z < 0:
		color_tuple = (255, 0, 0)

	pygame.draw.circle(pygame.display.get_surface(), color_tuple, (p_pix_x, p_pix_y), 3, 0)
	pygame.display.update()

def handle_goal_point(msg):
	global S
	g_pix_x = int(350 + msg.x * 35)
	g_pix_y = int(350 - msg.y * 35)
	pygame.draw.circle(pygame.display.get_surface(), (0, 255, 0), (g_pix_x, g_pix_y), 3, 0)
	pygame.display.update()

if __name__ == '__main__':
	rospy.init_node('MapTree_visual')

	S.fill((255, 255, 255))
	rospy.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates, handle_model_states)
	rospy.Subscriber('sampled_point', hector_uav_msgs.msg.Vector, handle_sampled_point)
	# rospy.Subscriber('sampled_point', hector_uav_msgs.msg.Vector, handle_sampled_point2)
	rospy.Subscriber('actual_uav_goal', hector_uav_msgs.msg.Vector, handle_goal_point);
	pygame.display.flip()

	running = True
	while not rospy.is_shutdown() and running:
		if total_points == 300:
			rospy.loginfo('Sampling is complete.')
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False

	rospy.spin()