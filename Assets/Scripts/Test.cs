using UnityEngine;
using System.Collections.Generic;
using System;
using Unity.Mathematics;

public class Test : MonoBehaviour
{
    [Header("Objects")] 
    public Transform waypointsParent; // Empty Object (Waypoints Grubu)
    private List<Transform> waypoints = new List<Transform>();
    private int targetIndex = 0;
    private int nextTargetIndex = 0;

    [Header("Collider")]
    public WheelCollider frontLeftCollider;
    public WheelCollider frontRightCollider;
    public WheelCollider rearLeftCollider;
    public WheelCollider rearRightCollider;

    

    [Header("Mesh")]
    public Transform frontLeftMesh;
    public Transform frontRightMesh;
    public Transform rearLeftMesh;
    public Transform rearRightMesh;

    public float torque;

    [Header("Araç ayarları")]
    public float lookAheadDistance = 5.0f; // Bakış mesafesi
    public float maxSpeed = 10.0f; // Maksimum hız
    public float steeringSmoothness = 1f;
    public float maxSteeringAngle = 30f;
    public float wheelBase = 2.5f; // Araç dingil mesafesi


    [Header("PID Ayarları")]
    public float Kp = 1.0f;
    public float Ki = 0.0f;
    public float Kd = 0.1f;

    private float steerIntegral = 0f;
    private float lastSteerError = 0f;

    [Header("turn")]
    public bool isTurning = false;
    
    public float currentSteerAngle = 0f;
    public float pidOutput;
    

    void Start()
    {
        // Waypointleri çocuk nesnelerden al
        foreach (Transform child in waypointsParent)
        {
            waypoints.Add(child);
        }
        Debug.Log(waypoints.Count);
        OnDrawGizmos();
    }

    void Update()
    {
        if (waypoints.Count == 0) return;

        // En uygun hedef noktayı bul
        Vector3 target = waypoints[FindLookAheadPoint()].position;
        Vector3 nextTarget = waypoints[FindLookAheadPoint()+1].position;
        
        float currentSteerAngle = CalculateSteeringAngle(target);
        
       
        
        pidOutput = PIDSteering(currentSteerAngle, frontLeftCollider.steerAngle, Time.deltaTime);
        if(Mathf.Abs(pidOutput) >= 25.0f){
            isTurning = true;
        }
        else{
            isTurning = false;
        }
       

        MoveCar(torque, isTurning);
        ApplySteering(pidOutput); 
        UpdateWheelRotation(frontLeftCollider, frontLeftMesh);
        UpdateWheelRotation(frontRightCollider, frontRightMesh);
    }

    int FindLookAheadPoint()
    {
        while (targetIndex < waypoints.Count - 1)
        {
            float distance = Vector3.Distance(transform.position, waypoints[targetIndex].position);

            if (targetIndex == 44)
            {
                targetIndex = 0;
            }

            if (distance < lookAheadDistance)
            {
                Debug.Log(targetIndex);
                targetIndex++;
            }
            
            else
            {
                break;
            }
        }

        return targetIndex;
    }

    float CalculateSteeringAngle(Vector3 target)
    {
        Vector3 localTarget = transform.InverseTransformPoint(target);
        float alpha = Mathf.Atan2(localTarget.x, localTarget.z);
        float L = Mathf.Sqrt(localTarget.x * localTarget.x + localTarget.y * localTarget.y + localTarget.z * localTarget.z);
        float delta = Mathf.Atan2(2.0f * wheelBase * Mathf.Sin(alpha) / L, 1.0f);

        return delta * Mathf.Rad2Deg;
    }

     bool CheckIfTurning()
    {
        if (targetIndex >= waypoints.Count - 1) return false;
        Vector2 current = new Vector2(waypoints[targetIndex+1].position.x,waypoints[targetIndex+1].position.z).normalized;
        Vector2 next = new Vector2(waypoints[targetIndex+2].position.x,waypoints[targetIndex+2].position.z).normalized;
        float dy = next.y - current.y;
        float dx = next.x - current.x;
        float angle = Mathf.Atan(dy/dx);
        Debug.Log(Mathf.Abs(angle));
        return Mathf.Abs(angle) >= 1;


        
        
    }
    void ApplySteering(float targetAngle)
    {
        currentSteerAngle = Mathf.Lerp(currentSteerAngle, targetAngle, Time.deltaTime * steeringSmoothness);
        frontLeftCollider.steerAngle = currentSteerAngle;
        frontRightCollider.steerAngle = currentSteerAngle;
    }

    void MoveCar(float torque, bool applyBrake)
    {
    
        if(isTurning){
            rearLeftCollider.motorTorque = torque/2;
            rearRightCollider.motorTorque = torque/2;        
        }
        else {
            
            rearLeftCollider.motorTorque = torque;
            rearRightCollider.motorTorque = torque;        
        
        }
    }

    void UpdateWheelRotation(WheelCollider collider, Transform wheelMesh)
    {
        Vector3 wheelPos;
        Quaternion wheelRot;
        collider.GetWorldPose(out wheelPos, out wheelRot);

        wheelMesh.position = wheelPos;
        wheelMesh.rotation = wheelRot;

        float rpm = collider.rpm;
        wheelMesh.Rotate(Vector3.right, rpm * 360 * Time.deltaTime);
    }
    void OnDrawGizmos()
    {
        if (waypointsParent == null) return;

        Gizmos.color = Color.green;

        // Tüm waypoint noktalarını çiz
        for (int i = 0; i < waypointsParent.childCount; i++)
        {
            Transform wp = waypointsParent.GetChild(i);
            if (wp != null)
            {
                Gizmos.DrawSphere(wp.position, 0.1f); // Küre boyutu 0.5
            }

            // İki waypoint arası çizgi çiz
            if (i < waypointsParent.childCount - 1)
            {
                Transform next = waypointsParent.GetChild(i + 1);
                Gizmos.DrawLine(wp.position, next.position);
            }
        }

        // Döngü tamamlanıyorsa son noktayı ilk noktayla birleştir
        if (waypointsParent.childCount > 1)
        {
            Transform first = waypointsParent.GetChild(0);
            Transform last = waypointsParent.GetChild(waypointsParent.childCount - 1);
            Gizmos.DrawLine(last.position, first.position);
        }
    }
    float PIDSteering(float targetAngle, float currentAngle, float deltaTime)
    {
        float error = Mathf.DeltaAngle(currentAngle, targetAngle);
        steerIntegral += error * deltaTime;
        float derivative = (error - lastSteerError) / deltaTime;

        float output = Kp * error + Ki * steerIntegral + Kd * derivative;

        lastSteerError = error;

    // Direksiyon sınırlandır
        return Mathf.Clamp(output, -maxSteeringAngle, maxSteeringAngle);
    }
}
