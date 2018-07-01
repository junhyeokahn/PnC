#pragma once

class TrajectoryParameter
{
public:
    TrajectoryParameter () : startTime(0), endTime(0);
    virtual ~TrajectoryParameter ();

private:
    /* data */
};

class Trajectory
{
public:
    Trajectory (arguments);
    virtual ~Trajectory ();

private:
    /* data */
};
