#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SpawnModel
# from geometry_msgs.msg import Quaternion, Pose, Point
import pika
import json
from RpcClient import RpcClient
from CoSpawner import CoSpawner
from multiprocessing import Process
from Point import Point, Vector2d
import PathPlanner as pp
from ORCA import ORCAsolver
from Heightmap import Heightmap
from targets_path_planning.msg import AllPaths, Path
from Point import Point as PointGeom, Vector2d
from geometry_msgs.msg import Pose, Twist, Quaternion, Point
from GridHeightmap import GridHeightmapHandler

credentials = pika.PlainCredentials('admin', 'admin')
connection = pika.BlockingConnection(pika.ConnectionParameters('192.168.1.65',
                                                               5672,
                                                               '/',
                                                               credentials, blocked_connection_timeout=0, heartbeat=0))

channel = connection.channel()

dict_of_robots = {}
grid_map_handler = GridHeightmapHandler()
channel.exchange_declare("co_create", exchange_type='topic', passive=False,
                         durable=False, auto_delete=False, arguments=None)
channel.exchange_declare("start_listening_co", exchange_type='topic', passive=False,
                         durable=False, auto_delete=False, arguments=None)

def spawn_co_rpc(ch, method, properties, body):

    print("here")
    recived_message = json.loads(body)
    print(recived_message)
    reg_new_co_in_db = RpcClient()
    try:
        print(recived_message)
        print(type(recived_message))
        print("here1")
        topleft = recived_message["TL"]
        bottomright = recived_message["BR"]
        print(topleft)
        print(bottomright)
        statuses = []
        for i in range(int(recived_message["num"])):
            result_from_creating_in_db = reg_new_co_in_db.call(json.dumps({'co_type': '1'}), "add_co_rpc")
            print(json.loads(result_from_creating_in_db))
            print(type(json.loads(result_from_creating_in_db)))
            result_from_creating_in_db = json.loads(result_from_creating_in_db)
            statuses.append(result_from_creating_in_db)
            co_spawner = CoSpawner(
                grid_map_handler = grid_map_handler,
                TL=topleft,
                BR=bottomright,
                id=result_from_creating_in_db["id"],
                waypoint=recived_message["waypoint"]
            )
            new_process = Process(co_spawner.spawn_robot())
            # co_spawner.spawn_robot()
            new_process.start()
            # dict_of_robots[result_from_creating_in_db["id"]] = CoSpawner
            # new_process.join()
        print(statuses)
        print("PPPPPPPPPPPPPP")
        channel.basic_publish(
            exchange='co_create',
            routing_key="",
            body=json.dumps(statuses),
            properties=pika.BasicProperties(
                delivery_mode=2,
            ))
        ch.basic_publish(exchange='',
                         routing_key=properties.reply_to,
                         properties=pika.BasicProperties(correlation_id= \
                                                             properties.correlation_id),
                         body=json.dumps(statuses))


    except KeyError:
        statuses = []
        for i in range(int(recived_message["num"])):
            result_from_creating_in_db = reg_new_co_in_db.call(json.dumps({'co_type': '1'}), "add_co_rpc")
            result_from_creating_in_db = json.loads(result_from_creating_in_db)
            statuses.append(result_from_creating_in_db)
            co_spawner = CoSpawner(
                grid_map_handler=grid_map_handler,
                spawn_point=recived_message["point"],
                id=result_from_creating_in_db["id"],
                waypoint=recived_message["waypoint"]
            )
            new_process = Process(co_spawner.spawn_robot())
            new_process.start()
            # new_process.join()
            dict_of_robots[result_from_creating_in_db["id"]] = CoSpawner
        ch.basic_publish(exchange='',
                         routing_key=properties.reply_to,
                         properties=pika.BasicProperties(correlation_id= \
                                                             properties.correlation_id),
                         body=json.dumps(statuses))
        print("PPPPPPPPP")
        channel.basic_publish(
            exchange='co_create',
            routing_key="",
            body=json.dumps(statuses),
            properties=pika.BasicProperties(
                delivery_mode=2,
            ))

    except Exception as e:
        final_json = {"error": "error"}
        print(e)
        ch.basic_publish(exchange='',
                         routing_key=properties.reply_to,
                         properties=pika.BasicProperties(correlation_id= \
                                                             properties.correlation_id),
                         body=json.dumps(final_json))


def set_co_waypoint_rpc(ch, method, properties, body):
    try:
        recived_message = json.loads(body)
        print("here")
        print(recived_message)
        print("here2")
        print(dict_of_robots)
        print("here3")
        print(dict_of_robots[int(recived_message["id"])])
        print("here4")
        p = Point(int(recived_message["point"]["x"]), int(recived_message["point"]["y"]), 0)
        print("here5")
        dict_of_robots[int(recived_message["id"])].robot.waypoint_publisher(p)
        print("here6")
        status = {"status": "success"}
    except Exception as e:
        print(e)
        status = {"status": "failed"}
    ch.basic_publish(exchange='',
                     routing_key=properties.reply_to,
                     properties=pika.BasicProperties(correlation_id= \
                                                         properties.correlation_id),
                     body=json.dumps(status))

def prepare_path_msg( name, path):
        msg = Path()
        msg.path = []
        msg.robot_name = name
        for state in path:
            point = Point()
            point.x = state.x
            point.y = state.y
            point.z = state.z
            msg.path.append(point)
        return msg

def prepare_paths_msg(names, paths):
    msg = AllPaths()
    msg.path_list = []
    for name in names:
        path = paths[name]
        path_msg = prepare_path_msg(name, path)
        msg.path_list.append(path_msg)
    return msg

def start_moving(ch, method, properties, body):
    paths_pub = rospy.Publisher('all_paths_data', AllPaths, queue_size=10)
    channel.basic_publish(
        exchange='start_listening_co',
        routing_key="",
        body=json.dumps({"request": "start"}),
        properties=pika.BasicProperties(
            delivery_mode=2,
        ))
    paths = grid_map_handler.orca.run_orca()
    msg = prepare_paths_msg(paths.keys(), paths)
    paths_pub.publish(msg)

channel.queue_declare(queue='spawn_co_rpc', durable=False)
channel.basic_consume(queue='spawn_co_rpc', on_message_callback=spawn_co_rpc, auto_ack=True)
channel.queue_declare(queue='set_co_waypoint_rpc', durable=False)
channel.basic_consume(queue='set_co_waypoint_rpc', on_message_callback=set_co_waypoint_rpc, auto_ack=True)
channel.queue_declare(queue='start_moving_co', durable=False)
channel.basic_consume(queue='start_moving_co', on_message_callback=start_moving, auto_ack=True)
rospy.init_node('co_spawner')
channel.start_consuming()