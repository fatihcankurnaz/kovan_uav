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
	global prev_count
	try:
		n = len(msg.name)
		if n > prev_count:
			r_pix_x = int(350 + msg.pose[1].position.x * 35)
			r_pix_y = int(350 - msg.pose[1].position.y * 35)
			pygame.draw.circle(pygame.display.get_surface(), (255, 0, 0), (r_pix_x, r_pix_y), 3, 0)
			for i in range(2, n):
				o_pix_x = int(350 + msg.pose[i].position.x * 35)
				o_pix_y = int(350 - msg.pose[i].position.y * 35)
				pygame.draw.circle(pygame.display.get_surface(), (220, 220, 220, 100), (o_pix_x, o_pix_y), 34, 0)
			prev_count = n
	except IndexError as e:
		# raise e
		return
def handle_goal_point(msg):
	g_pix_x = int(350 + msg.x * 35)
	g_pix_y = int(350 - msg.y * 35)
	pygame.draw.circle(pygame.display.get_surface(), (0, 255, 0), (g_pix_x, g_pix_y), 6, 0)
	pygame.display.update()

def handle_sampled_point(msg):
	global end_1
	p_pix_x = int(350 + msg.x * 35)
	p_pix_y = int(350 - msg.y * 35)
	fg = 0, 0, 0
	font = pygame.font.Font(None, 20)
	text = "%.2f ,%.2f"% (msg.x,msg.y)
    	
	ren = font.render(text, 0, fg)
	pygame.draw.circle(pygame.display.get_surface(), (133, 0, 113), (p_pix_x, p_pix_y), 6, 0)
	if end_1 != (0,0):
		pygame.draw.line(pygame.display.get_surface(), (235, 231, 0), end_1, (p_pix_x, p_pix_y), 2)
	end_1 = (p_pix_x,p_pix_y)

	S.blit(ren,(p_pix_x,p_pix_y))
	pygame.display.flip()

if __name__ == '__main__':
	rospy.init_node('MapTree_visual')
	S.fill((255, 255, 255))
	rospy.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates, handle_model_states)
	rospy.Subscriber('sampled_point', hector_uav_msgs.msg.Vector, handle_sampled_point)
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
