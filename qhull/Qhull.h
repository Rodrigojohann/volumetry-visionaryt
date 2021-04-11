namespace orgQhull{
//...
private:
    PointCoordinates *m_externalPoints;
//...
public:
    void runQhull3D(const std::vector<vec3> &points, const char* args);
    void runQhull(const PointCoordinates &points, const char *qhullCommand2);
//...
}
