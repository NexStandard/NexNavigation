using Stride.Core.Mathematics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NexNavigation;
public class NavigationMap
{
    public Dictionary<Vector3, List<int>> Map { get; init; }
    public Dictionary<Vector3, BoundingBox> MappedBoundingBoxes { get; init; }
}
