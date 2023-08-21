
using Stride.Core.Mathematics;
using Stride.Physics;

namespace NexNavigation;
public class MapBuilder
{
    public NavigationMap BuildNavigationMap(StaticColliderComponent staticColliderComponent)
    {
        var convexHull = staticColliderComponent.ColliderShape as ConvexHullColliderShape;
        var indices = convexHull.Indices.Cast<int>().ToList();
        var vertices = convexHull.Points;
        Dictionary<Vector3, List<int>> navigationMap = new Dictionary<Vector3, List<int>>();

        for (int i = 0; i < indices.Count; i += 3)
        {
            int v1Index = indices[i];
            int v2Index = indices[i + 1];
            int v3Index = indices[i + 2];

            Vector3 v1 = vertices[v1Index];
            Vector3 v2 = vertices[v2Index];
            Vector3 v3 = vertices[v3Index];

            // Add the vertices to the navigation map if they don't exist
            if (!navigationMap.ContainsKey(v1))
                navigationMap.Add(v1, new List<int>());

            if (!navigationMap.ContainsKey(v2))
                navigationMap.Add(v2, new List<int>());

            if (!navigationMap.ContainsKey(v3))
                navigationMap.Add(v3, new List<int>());

            // Connect the vertices in the navigation map
            navigationMap[v1].Add(v2Index);
            navigationMap[v1].Add(v3Index);

            navigationMap[v2].Add(v1Index);
            navigationMap[v2].Add(v3Index);

            navigationMap[v3].Add(v1Index);
            navigationMap[v3].Add(v2Index);

            System.Console.WriteLine($"Connecting vertices: {v1Index} - {v2Index} - {v3Index}");
        }
        return new NavigationMap()
        {
            MappedBoundingBoxes = CalculateBoundingBoxesDictionaryFromVertices(vertices.ToArray()),
            Map = navigationMap
        };
    }
    private Dictionary<Vector3, BoundingBox> CalculateBoundingBoxesDictionaryFromVertices(Vector3[] vertices)
    {
        if (vertices == null || vertices.Length == 0)
        {
            return new ();
        }

        int maxVerticesPerBox = 3; // Adjust this based on your needs
        Dictionary<Vector3, BoundingBox> boundingBoxes = new Dictionary<Vector3, BoundingBox>();

        for (int i = 0; i < vertices.Length; i += maxVerticesPerBox)
        {
            int endIdx = Math.Min(i + maxVerticesPerBox, vertices.Length);
            Vector3[] subsetVertices = vertices.Skip(i).Take(endIdx - i).ToArray();

            BoundingBox boundingBox = CalculateBoundingBoxFromSubsetVertices(subsetVertices);

            foreach (Vector3 vertex in subsetVertices)
            {
                boundingBoxes[vertex] = boundingBox;
            }
        }

        return boundingBoxes;
    }
    private BoundingBox CalculateBoundingBoxFromSubsetVertices(Vector3[] vertices)
    {
        Vector3 minPoint = vertices[0];
        Vector3 maxPoint = vertices[0];

        for (int i = 1; i < vertices.Length; i++)
        {
            minPoint = Vector3.Min(minPoint, vertices[i]);
            maxPoint = Vector3.Max(maxPoint, vertices[i]);
        }

        return new BoundingBox(minPoint, maxPoint);
    }
}
public class NavigationAgentSettings
{
    public float Height;
    public float MaxClimb;
    public AngleSingle MaxSlope;
    public float Radius;
}