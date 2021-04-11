void Qhull::runQhull3D(const std::vector<vec3> &points, const char* args)
{
    m_externalPoints = new PointCoordinates(3);  //3 = dimension
    vector<double> allPoints;
    for each (vec3 p in points)
    {
        allPoints.push_back(p.x());
        allPoints.push_back(p.y());
        allPoints.push_back(p.z());
    }

    m_externalPoints->append(allPoints); //convert to vector<double>
    runQhull(*m_externalPoints, args);
}

void Qhull::runQhull(const PointCoordinates &points, const char *qhullCommand2)
{
    runQhull(points.comment().c_str(), points.dimension(), points.count(), &*points.coordinates(), qhullCommand2);
}
