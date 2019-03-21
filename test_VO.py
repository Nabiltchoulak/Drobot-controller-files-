import cv2

from visual_odometry import PinholeCamera, VisualOdometry

# Nabil abrikoulilna hna
cam = PinholeCamera(1241.0, 376.0, 718.8560, 718.8560, 607.1928, 185.2157)
vo = VisualOdometry(cam)

i = 0
while True:
	i += 1
	img = cv2.imread(TON_TRUC, 0)
	vo.update(img)
	cur_t = vo.cur_t
	if i > 2:
		x, y, z = cur_t[0], cur_t[1], cur_t[2]
		print(x, y, z)
	else:
		x, y, z = 0, 0, 0


	'''
for img_id in range(4541):
	img = cv2.imread('/home/xxx/datasets/KITTI_odometry_gray/00/image_0/'+str(img_id).zfill(6)+'.png', 0)

	vo.update(img, img_id)

	cur_t = vo.cur_t
	if(img_id > 2):
		x, y, z = cur_t[0], cur_t[1], cur_t[2]
		print(x, y, z)
	else:
		x, y, z = 0., 0., 0.
	'''
