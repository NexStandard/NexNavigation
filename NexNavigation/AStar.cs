using Stride.Core.Mathematics;
using Stride.Engine;
using System;

public class AStar
{
    public List<Vector3> FindPath(Dictionary<Vector3, List<int>> navigationMap, Dictionary<Vector3, BoundingBox> boundingBoxes, Vector3 start, Vector3 goal, float maxSlope)
    {
        BoundingBox goalBoundingBox;
        if (boundingBoxes.ContainsKey(goal))
        {
            goalBoundingBox = boundingBoxes[goal];
        }
        else
        {
            // Default bounding box if goal is not in the boundingBoxes dictionary
            goalBoundingBox = new BoundingBox(goal - new Vector3(0.5f), goal + new Vector3(0.5f));
        }

        PriorityQueue<Node> openSet = new PriorityQueue<Node>();
        HashSet<Vector3> closedSet = new HashSet<Vector3>();
        Dictionary<Vector3, Vector3> cameFrom = new Dictionary<Vector3, Vector3>();
        Dictionary<Vector3, float> gScore = new Dictionary<Vector3, float>();

        openSet.Enqueue(new Node(start, Heuristic(start, goalBoundingBox.Center),0f));

        gScore[start] = 0;

        while (openSet.Count > 0)
        {
            Node current = openSet.Dequeue();
            if (goalBoundingBox.Contains(ref goal) is not ContainmentType.Disjoint)
            {
                // Add the goal to the cameFrom dictionary to ensure it's included in the path
                cameFrom[goal] = current.Position;
                return ReconstructPath(cameFrom, goal);
            }

            closedSet.Add(current.Position);

            foreach (int neighborIndex in navigationMap[current.Position])
            {
                Vector3 neighbor = navigationMap.Keys.ToArray()[neighborIndex];
                if (closedSet.Contains(neighbor))
                    continue;

                BoundingBox neighborBoundingBox;
                if (boundingBoxes.ContainsKey(neighbor))
                {
                    neighborBoundingBox = boundingBoxes[neighbor];
                }
                else
                {
                    neighborBoundingBox = new BoundingBox(neighbor - new Vector3(0.5f), neighbor + new Vector3(0.5f));
                }

                if (!IsSlopeAcceptable(current.Position, neighbor, maxSlope) || !IsBoundingBoxValid(current.Position, neighborBoundingBox))
                    continue;

                float tentativeGScore = gScore[current.Position] + Distance(current.Position, neighbor);

                if (!gScore.ContainsKey(neighbor) || tentativeGScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = current.Position;
                    gScore[neighbor] = tentativeGScore;

                    float fScore = tentativeGScore + Heuristic(neighbor, goalBoundingBox.Center);
                    openSet.Enqueue(new Node(neighbor, fScore, tentativeGScore));
                }
            }
        }

        return null; // Path not found
    }

    private bool IsSlopeAcceptable(Vector3 from, Vector3 to, float maxSlope)
    {
        // Calculate the vertical change (rise) and horizontal distance (run)
        float verticalChange = to.Y - from.Y;
        float horizontalDistance = Vector2.Distance(new Vector2(from.X, from.Z), new Vector2(to.X, to.Z));

        // Calculate the slope angle in radians
        float slopeAngleRad = (float)Math.Atan(verticalChange / horizontalDistance);

        // Convert the slope angle to degrees
        float slopeAngleDeg = (float)(slopeAngleRad * (180f / Math.PI));

        // Check if the absolute value of the slope angle is within the acceptable range
        return Math.Abs(slopeAngleDeg) <= maxSlope;
    }
    private bool IsBoundingBoxValid(Vector3 from, BoundingBox boundingBox)
    {    
        Ray ray = new Ray(from, boundingBox.Center - from);
        return ray.Intersects(ref boundingBox);
    }
    private List<Vector3> ReconstructPath(Dictionary<Vector3, Vector3> cameFrom, Vector3 current)
    {
        List<Vector3> path = new List<Vector3>();
        while (cameFrom.ContainsKey(current))
        {
            path.Add(current);
            current = cameFrom[current];
        }
        path.Reverse();
        return path;
    }

    private float Distance(Vector3 a, Vector3 b)
    {
        return Vector3.Distance(a, b);
    }

    private float Heuristic(Vector3 a, Vector3 b)
    {
        return Distance(a, b);
    }

    private class Node : IComparable<Node>
    {
        public Vector3 Position { get; }
        public float FScore { get; }
        public float GScore { get; }

        public Node(Vector3 position, float fScore, float gScore)
        {
            Position = position;
            FScore = fScore;
            GScore = gScore;
         
        }

        public int CompareTo(Node other)
        {
            return FScore.CompareTo(other.FScore);
        }
    }

    private class PriorityQueue<T> where T : IComparable<T>
    {
        private List<T> items = new List<T>();

        public int Count => items.Count;

        public void Enqueue(T item)
        {
            items.Add(item);
            items.Sort();
        }

        public T Dequeue()
        {
            T item = items[0];
            items.RemoveAt(0);
            return item;
        }
    }
}