
using Stride.Core.Mathematics;


namespace NexNavigation;
public class MapBuilder
{
    public Dictionary<Vector3, List<int>> BuildNavigationMap(List<Vector3> vertices, List<int> indices)
    {
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

        return navigationMap;
    }
}
public class NavigationAgentSettings
{
    public float Height;
    public float MaxClimb;
    public AngleSingle MaxSlope;
    public float Radius;
}