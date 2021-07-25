using Stride.Core;
using Stride.Engine;
using Stride.Engine.Design;
using QuikGraph;
using QuikGraph.Algorithms;
using QuikGraph.Utils;
using QuikGraph.Predicates;
using Stride.Rendering;
using System.Linq;
using System.Collections.Generic;
using QuikGraph.Algorithms.ShortestPath;
using Stride.Core.Mathematics;
using Stride.Games;
using System;

namespace IKTesting
{
    [DataContract("IKComponent")]
    [DefaultEntityComponentProcessor(typeof(IKProcessor), ExecutionMode = ExecutionMode.Runtime | ExecutionMode.Thumbnail | ExecutionMode.Preview)]
    [Display("Inverse Kinematics", Expand = ExpandRule.Once)]
    [ComponentOrder(2005)]
    [ComponentCategory("Animation")]
    public class IKComponent : EntityComponent
    {
        private AdjacencyGraph<NodeData,Edge<NodeData>> _graph;

        private IVertexAndEdgeListGraph<NodeData,Edge<NodeData>> Graph 
        {
            get {return _graph;}
        }
        private TryFunc<NodeData, IEnumerable<Edge<NodeData>>> ShortestPath;
        private List<NodeData> Leaves;

        [DataMember("Number of iteration")]
        public uint NbIteration;
        public Dictionary<string,Entity> BoneToTarget = new();
        
        public void BuildGraph()
        {
            _graph = new();
            var sk = Entity.Get<ModelComponent>().Skeleton;
            var nodes = sk.Nodes.Select((x,i) => new NodeData{Index = i, Name = x.Name, Parent = x.ParentIndex, Node = sk.NodeTransformations[i]}).ToList();
            nodes.ForEach(
                x =>
                {
                    var parent = _graph.Vertices.FirstOrDefault(e => e.Index == x.Parent);
                    // skGraph.AddVertex(x);
                    if(parent != null)
                    {
                        _graph.AddVertex(x);
                        parent.Distance = Vector3.Distance(parent.Node.WorldMatrix.TranslationVector,x.Node.WorldMatrix.TranslationVector); //Vector3.Distance(parent.Node.WorldMatrix.TranslationVector, x.Node.WorldMatrix.TranslationVector);
                        _graph.AddEdge(new Edge<NodeData>(parent,x));
                    }
                    else
                    {
                        _graph.AddVertex(x);
                    }
                }
            );
            Leaves = _graph.Vertices.Where(x => !_graph.Edges.Any(e => e.Source.Index == x.Index)).Where(x => BoneToTarget.Keys.Contains(x.Name)).ToList();
            ShortestPath = Graph.ShortestPathsDijkstra(_ => 1, _graph.Vertices.First());
        }

        public void ComputeFabrik(GameTime time)
        {
            
            foreach(var (n,e) in BoneToTarget)
            {
                var md = Entity.Get<ModelComponent>();
                ShortestPath(_graph.Vertices.First(x => x.Name == n), out var p);
                var d = p.Sum(x => x.Source.Distance);
                var sk = Entity.Get<ModelComponent>().Skeleton;
                var currNode = sk.NodeTransformations[4];
                // var target = WorldToLocal(e.Transform, currNode);
                // var direction = target - currNode.Transform.Position; 
                Vector3 target = Vector3.TransformCoordinate( e.Transform.WorldMatrix.TranslationVector ,Matrix.Invert(currNode.WorldMatrix));
                // sk.NodeTransformations[10].Transform.Rotation *= -Quaternion.RotationY(30 * (float)time.Elapsed.TotalSeconds);
                sk.NodeTransformations[4].Transform.Position = target;
                
            }
            
        }
        public Vector3 WorldToLocal(TransformComponent tc, ModelNodeTransformation nt)
        {
            Matrix.Invert(ref nt.WorldMatrix, out Matrix worldMatrixInv);
            tc.GetWorldTransformation(out Vector3 p, out Quaternion r, out Vector3 s);
            Vector3.Transform(ref p, ref worldMatrixInv, out Vector3 result);
            return result;
        }
        public Vector3 WorldToLocal(Vector3 p, ModelNodeTransformation nt)
        {
            Matrix.Invert(ref nt.WorldMatrix, out Matrix worldMatrixInv);
            Vector3.TransformCoordinate(ref p, ref worldMatrixInv, out Vector3 result);
            return result;
        }
        public Quaternion DirRotation(Vector3 dir)
        {
            var m = Matrix.LookAtRH(Vector3.Zero, dir, Vector3.UnitY);
            return Quaternion.RotationMatrix(m);
        }
        public Matrix LookAt(Vector3 dir)
        {
            return Matrix.LookAtRH(Vector3.Zero, dir, Vector3.UnitY);
            
        }
        public bool CheckValid()
        {
            var mc = Entity.Get<ModelComponent>();
            return mc != null && mc.Skeleton != null;
        }
        internal class NodeData
        {
            public int Index {get;set;}
            public string Name {get;set;}
            public int Parent {get;set;}
            public float Distance {get;set;}
            public ModelNodeTransformation Node {get;set;}
        }
        public class FabrikData
        {
            public Vector3 Position;
            public Quaternion Rotation;
            public float Distance;
        }
    }
}