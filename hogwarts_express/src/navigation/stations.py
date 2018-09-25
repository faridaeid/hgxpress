from enum import Enum
from geometry_msgs.msg import Point, Pose, Quaternion

class Station(Enum):
    Platform = 0
    Gringotts = 1
    TheBurrow = 2
    DiagonAly = 3
    ForbiddedForest = 4
    Hogwarts = 5
    Hogsmeade = 6
    Ministry = 7
    Azkaban = 8


class StationDirection(Enum):
    PlatformFor = 0
    PlatformBack = 1
    GringottsFor = 2
    GringottsBack = 3
    TheBurrowFor = 4
    TheBurrowBack = 5
    DiagonAlyFor = 6
    DiagonAlyBack = 7
    ForbiddedForestFor = 8
    ForbiddedForestBack = 9
    HogwartsFor = 10
    HogwartsBack = 11
    HogsmeadeFor = 12
    HogsmeadeBack = 13
    MinistryFor = 14
    MinistryBack = 15
    AzkabanFor = 16
    AzkabanBack = 17
    PlatformPlatformCorr = 18
    BurrBurrCorr = 19
    GringDiagCorr = 20
    DiagMinisCorr = 21


request_mapping = {
    Station.Platform: StationDirection.PlatformBack,
    Station.Gringotts: StationDirection.GringottsBack,
    Station.TheBurrow: StationDirection.TheBurrowBack,
    Station.DiagonAly: StationDirection.DiagonAlyBack,
    Station.ForbiddedForest: StationDirection.ForbiddedForestBack,
    Station.Hogwarts: StationDirection.HogwartsBack,
    Station.Hogsmeade: StationDirection.HogsmeadeBack,
    Station.Ministry: StationDirection.MinistryBack,
    Station.Azkaban: StationDirection.AzkabanBack
}

destination_mapping = {
    Station.Platform: StationDirection.PlatformFor,
    Station.Gringotts: StationDirection.GringottsFor,
    Station.TheBurrow: StationDirection.TheBurrowFor,
    Station.DiagonAly: StationDirection.DiagonAlyFor,
    Station.ForbiddedForest: StationDirection.ForbiddedForestFor,
    Station.Hogwarts: StationDirection.HogwartsFor,
    Station.Hogsmeade: StationDirection.HogsmeadeFor,
    Station.Ministry: StationDirection.MinistryFor,
    Station.Azkaban: StationDirection.AzkabanFor
}

corrections = {
    (StationDirection.PlatformBack, StationDirection.PlatformFor):
        StationDirection.PlatformPlatformCorr,
    (StationDirection.TheBurrowFor, StationDirection.TheBurrowBack):
        StationDirection.BurrBurrCorr,
    (StationDirection.GringottsBack, StationDirection.DiagonAlyFor):
        StationDirection.GringDiagCorr,
    (StationDirection.DiagonAlyFor, StationDirection.MinistryBack):
        StationDirection.DiagMinisCorr

    # (StationDirection.ForbiddedForestFor, StationDirection.ForbiddedForestBack):
    #     StationDirection.ForbiddenForestCorrection,
    # (StationDirection.HogsmeadeBack, StationDirection.HogsmeadeFor):
    #     StationDirection.HogsmeadeCorrection,
    # (StationDirection.AzkabanFor, StationDirection.AzkabanBack):
    #     StationDirection.AzkabanCorrection,
    # (StationDirection.DiagonAlyFor, StationDirection.DiagonAlyBack):
    #     StationDirection.DiagonAlyFor
}


station_poses = {
    StationDirection.PlatformBack: Pose(Point(96.072, 94.425, 0.000), Quaternion(0.000, 0.000, -0.825, 0.564)),
    StationDirection.PlatformFor: Pose(Point(97.476, 94.183, 0.000), Quaternion(0.000, 0.000, 0.449, 0.8940)),
    StationDirection.GringottsFor: Pose(Point(100.123, 94.609, 0.000), Quaternion(0.000, 0.000, -0.175, 0.985)),
    StationDirection.GringottsBack: Pose(Point(100.056, 95.646, 0.000), Quaternion(0.000, 0.000, 0.985, 0.171)),
    StationDirection.TheBurrowFor: Pose(Point(101.887, 92.529, 0.000), Quaternion(0.000, 0.000, -0.177, 0.984)),
    StationDirection.TheBurrowBack: Pose(Point(101.802, 93.723, 0.000), Quaternion(0.000, 0.000, 0.983, 0.182)),
    StationDirection.DiagonAlyFor: Pose(Point(97.966, 97.507, 0.000), Quaternion(0.000, 0.000, 0.567, 0.824)),
    StationDirection.DiagonAlyBack: Pose(Point(96.685, 97.749, 0.000), Quaternion(0.000, 0.000, -0.830, 0.558)),
    StationDirection.ForbiddedForestFor: Pose(Point(104.704, 94.268, 0.000), Quaternion(0.000, 0.000, 0.584, 0.812)),
    StationDirection.ForbiddedForestBack: Pose(Point(103.454, 94.539, 0.000), Quaternion(0.000, 0.000, 0.566, 0.825)),
    StationDirection.HogwartsFor: Pose(Point(103.655, 96.656, 0.000), Quaternion(0.000, 0.000, 0.577, 0.817)),
    StationDirection.HogwartsBack: Pose(Point(102.446, 97.140, 0.000), Quaternion(0.000, 0.000, -0.820, 0.572)),
    StationDirection.HogsmeadeFor: Pose(Point(105.018, 98.331, 0.000), Quaternion(0.000, 0.000, 0.882, 0.471)),
    StationDirection.HogsmeadeBack: Pose(Point(105.004, 98.204, 0.000), Quaternion(0.000, 0.000, 0.797, 0.605)),
    StationDirection.MinistryFor: Pose(Point(101.139, 99.617, 0.000), Quaternion(0.000, 0.000, 0.981, 0.195)),
    StationDirection.MinistryBack: Pose(Point(101.519, 100.232, 0.000), Quaternion(0.000, 0.000, -0.186, 0.983)),
    StationDirection.AzkabanFor: Pose(Point(98.788, 100.315, 0.000), Quaternion(0.000, 0.000, 0.998, -0.056)),
    StationDirection.AzkabanBack: Pose(Point(99.453, 100.078, 0.000), Quaternion(0.000, 0.000, 1.000, 0.023)),
    StationDirection.PlatformPlatformCorr: Pose(Point(96.084, 93.343, 0.000), Quaternion(0.000, 0.000, -0.149, 0.989)),
    StationDirection.BurrBurrCorr: Pose(Point(102.717, 92.775, 0.000), Quaternion(0.000, 0.000, 0.632, 0.775)),
    StationDirection.GringDiagCorr: Pose(Point(98.573, 96.086, 0.000), Quaternion(0.000, 0.000, 0.929, 0.370)),
    StationDirection.DiagMinisCorr: Pose(Point(99.280, 99.194, 0.000), Quaternion(0.000, 0.000, 0.326, 0.945))
}
