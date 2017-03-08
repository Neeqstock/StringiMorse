def print_pos(pos):
    print("X = %s, Y = %s and yaw =%s" % (pos['x'], pos['y'], pos['yaw']))

    # only displaying useful information


def print_prox(proxi):
    print("target = %s" % proxi['near_objects']['target'])


def print_ir(ir):
    print("left = %s, center = %s, right = %s" % (ir['range_list'][20], ir['range_list'][10], ir['range_list'][0]))

    # range 20 is the leftmost, 0 the rightmost


def move(v, w=0):
    motion.publish({"v": v, "w": w})


def stop():
    motion.publish({"v": 0, "w": 0})