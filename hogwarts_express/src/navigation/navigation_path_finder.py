#!usr/bin/env python

from stations import StationDirection, \
    request_mapping, destination_mapping, \
    Station, corrections
import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray

edges = {
    StationDirection.PlatformFor: [StationDirection.GringottsFor,
                                   StationDirection.DiagonAlyFor],
    StationDirection.PlatformBack: [StationDirection.PlatformFor],
    StationDirection.GringottsFor: [StationDirection.TheBurrowFor,
                                    StationDirection.ForbiddedForestBack],
    StationDirection.GringottsBack: [StationDirection.PlatformBack,
                                     StationDirection.DiagonAlyFor],
    StationDirection.TheBurrowFor: [StationDirection.ForbiddedForestBack,
                                    StationDirection.TheBurrowBack],
    StationDirection.TheBurrowBack: [StationDirection.GringottsBack],
    StationDirection.DiagonAlyFor: [StationDirection.MinistryBack],
    StationDirection.DiagonAlyBack: [StationDirection.PlatformBack,
                                     StationDirection.GringottsFor,
                                     StationDirection.DiagonAlyFor],
    StationDirection.ForbiddedForestBack: [StationDirection.HogwartsFor],
    StationDirection.HogwartsFor: [StationDirection.HogsmeadeBack,
                                   StationDirection.MinistryFor],
    StationDirection.HogwartsBack: [StationDirection.GringottsBack],
    StationDirection.HogsmeadeBack: [StationDirection.MinistryFor],
    StationDirection.MinistryFor: [StationDirection.AzkabanBack],
    StationDirection.MinistryBack: [StationDirection.HogwartsBack],
    StationDirection.AzkabanBack: [StationDirection.DiagonAlyBack]
}


class PathFinder(object):
    def __init__(self):
        self.path_publisher = rospy.Publisher('navigation/path', Int32MultiArray)

    def _get_shortest_path(self, dest):
        path = []
        path.insert(0, dest)
        pred_index = dest.value
        while self.pred[pred_index] != -1:
            path.insert(0, StationDirection(self.pred[pred_index]))
            pred_index = self.pred[pred_index]
        return path

    def _get_path(self, source, dest):
        visited = np.full(18, False, dtype=bool)
        self.pred = np.full(18, -1, dtype=int)
        visited[source.value] = True
        queue = []
        queue.insert(0, source)

        while len(queue):
            curr_vertix = queue.pop()
            for neighbor in edges[curr_vertix]:
                if not visited[neighbor.value]:
                    visited[neighbor.value] = True
                    self.pred[neighbor.value] = curr_vertix.value
                    queue.insert(0, neighbor)

                    if neighbor == dest:
                        return self._get_shortest_path(dest)

    def _recursion(self, source):
        self.visited[source] = True
        min_length = 99
        final_path = []

        for index, request in enumerate(self.requests):
            if not self.visited[request]:
                path = self._get_path(source, request)

                path += self._recursion(request)[1:]

                if len(path) < min_length:
                    min_length = len(path)
                    final_path = path

        self.visited[source] = False
        return final_path

    def _tweaked_bfs(self, source, requests):
        self.visited = {}
        self.visited[source] = False
        self.requests = requests
        for request in requests:
            self.visited[request] = False
        min_path = self._recursion(source)

        for index in np.arange(len(min_path) -1):
            if (min_path[index], min_path[index+1]) in corrections:
                min_path.insert(index+1, corrections[(min_path[index], min_path[index+1])])
        return min_path

    def publish_path(self, source, requests):
        # reqs = [request_mapping[request] for request in requests]
        path = self._tweaked_bfs(source, requests)
        path.append(edges[path[-1]][0])
        print "Path generated is : ", path
        path_array = Int32MultiArray()
        path_array.data = [station.value for station in path]
        self.path_publisher.publish(path_array)



if __name__ == '__main__':
    path_finder = PathFinder()
    rospy.spin()

