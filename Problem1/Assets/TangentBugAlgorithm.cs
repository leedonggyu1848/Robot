using System.Collections.Generic;
using UnityEngine;

public class TangentBugAlgorithm : MonoBehaviour
{
    // input values
    public Transform target;
    
    // constants
    private const float SameDistanceThreshold = 1f; // 같은 위치로 판단할 거리
    private const float SensorRange = 2f; // 센서 범위
    private const float MoveSpeed = 1f; // 이동 속도
    private const float SensorInterval = 0.5f; // 센서 감지 간격
    
    // values
    private MoveState _state = MoveState.Move; // 현재 행동 상태
    private float _minHeuristicDistance = float.MaxValue;
    private float _minDistance = float.MaxValue;
    

    private enum MoveState
    {
        Move, // move to target
        Boundary // boundary following
    }

    private struct RayState
    {
        public Vector3 point;
        public bool IsCrash;
    }
    
    void Start()
    {
        RotationTo(Target());
    }

    // Update is called once per frame
    void Update()
    {
        if (IsSamePoint(Target(), Current())) FinishSimulation();
        transform.Translate(MoveSpeed * Time.deltaTime * Vector3.forward);
        
        switch (_state)
        {
            case MoveState.Move:
                MotionToGoal();
                break;
            case MoveState.Boundary:
                BoundaryFollowing();
                break;
        }
    }
    
    private void BoundaryFollowing()
    {
        var layover = FindMinimumLayover(out _, out var leaveDistance);

        if (leaveDistance < _minDistance)
        {
            _state = MoveState.Move;
            _minDistance = leaveDistance;
        }
        else
        {
            RotationTo(layover);
        }
    }

    private void MotionToGoal()
    {
        var layover = FindMinimumLayover(out var hitCount, out var minDistance);
        _minDistance = Mathf.Min(minDistance, _minDistance);
        
        var isUpdate = false;
        if (hitCount <= 0)
        {
            RotationTo(T());
            return;
        }

        var distance = GetDistanceToTargetWith(layover);
        if (distance < _minHeuristicDistance)
        {
            _minHeuristicDistance = distance;
            isUpdate = true;
        }
        
        if (CanGoT())
        {
            if (GetDistanceToTargetWith(T()) < distance)
            {
                RotationTo(T());
                _minHeuristicDistance = float.MaxValue;
                return;
            }
        }
        RotationTo(layover);

        if (isUpdate) return;
        _state = MoveState.Boundary;
        _minHeuristicDistance = float.MaxValue;
    }

    private Vector3 FindMinimumLayover(out int hitCount, out float minDistance)
    {
        var shortest = float.MaxValue;
        var rays = RayCircleObstacle();
        var bf = rays[^1];
        var ret = Vector3.zero;
        hitCount = 0;
        minDistance = float.MaxValue;

        foreach (var ray in rays)
        {
            if (ray.IsCrash)
                minDistance = Mathf.Min(minDistance, Vector3.Distance(Target(), ray.point));
            if (ray.IsCrash != bf.IsCrash)
            {
                hitCount++;
                var candidate = ray.IsCrash ? bf.point : ray.point;
                var curDistance = GetDistanceToTargetWith(candidate);
                if (curDistance < shortest)
                {
                    shortest = curDistance;
                    ret = candidate;
                }
            }
            bf = ray;   
        }

        return ret;
    }

    private float GetDistanceToTargetWith(Vector3 point)
    {
        return Vector3.Distance(Current(), point) + Vector3.Distance(point, Target());
    }

    private bool CanGoT()
    {
        return !RaycastObstacle(Target() - Current(), out _);
    }
    
    private Vector3 T()
    {
        var direction = (Target() - Current()).normalized;
        return Current() + direction * SensorRange;
    }
        
    private List<RayState> RayCircleObstacle()
    {
        var direction = Vector3.forward;
        var ret = new List<RayState>();
        RayState v;
        RaycastHit hit;
        for (float angle = 0; angle < 360; angle += SensorInterval)
        {
            direction = Quaternion.AngleAxis(SensorInterval, Vector3.up) * direction;
            v.IsCrash = RaycastObstacle(direction, out hit);
            if (v.IsCrash)
                v.point = hit.point;
            else v.point = Current() + direction * SensorRange;
            ret.Add(v);
        }
        return ret;
    }
    
    private bool RaycastObstacle(Vector3 direction, out RaycastHit hit)
    {
        var ret =  Physics.Raycast(Current(), direction, out hit, SensorRange);
        return ret && hit.transform.CompareTag("Obstacle");
    }

    private void RotationTo(Vector3 target)
    {
        target.y = Target().y;
        transform.LookAt(target);
    }
    
    private void FinishSimulation()
    {
        gameObject.SetActive(false);
    }
    
    private bool IsSamePoint(Vector3 a, Vector3 b)
    {
        return Vector3.Distance(a, b) < SameDistanceThreshold;
    }
    
    private Vector3 Target()
    {
        return target.transform.position;
    }

    private Vector3 Current()
    {
        return transform.position;
    }

}
