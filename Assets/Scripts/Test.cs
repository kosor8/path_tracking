using UnityEngine;
using System.Collections.Generic;

public class Test : MonoBehaviour
{
    public Transform waypointsParent; // Empty Object (Waypoints Grubu)
    private List<Transform> waypoints = new List<Transform>();

    public float lookAheadDistance = 5.0f; // Bakış mesafesi
    public float maxSpeed = 10.0f; // Maksimum hız
    private float currentSpeed; // Mevcut hız

    public WheelCollider frontLeftCollider;
    public WheelCollider frontRightCollider;
    public float torque;
    

    public float wheelBase = 2.5f; // Araç dingil mesafesi
    private int targetIndex = 0;

    public float steeringAngle;

    private float turnStartTime; // Dönüş başlama zamanı
    private bool isTurning = false; // Dönüş durumunu kontrol et

    void Start()
    {
        // Başlangıç hızını ayarla
        currentSpeed = maxSpeed;

        // Waypointleri çocuk nesnelerden al
        foreach (Transform child in waypointsParent)
        {
            waypoints.Add(child);
        }
    }

    void Update()
    {
        if (waypoints.Count == 0) return;

        // En uygun hedef noktayı bul
        Vector3 target = FindLookAheadPoint();

        // Pure Pursuit dönüş açısını hesapla
        steeringAngle = CalculateSteeringAngle(target);

    
        ApplyTest(steeringAngle);
    }

    Vector3 FindLookAheadPoint()
    {
        while (targetIndex < waypoints.Count - 1)
        {
            float distance = Vector3.Distance(transform.position, waypoints[targetIndex].position);
            if (distance < lookAheadDistance)
                targetIndex++;
            else
                break;
        }
        return waypoints[targetIndex].position;
    }

    float CalculateSteeringAngle(Vector3 target)
    {
        Vector3 localTarget = transform.InverseTransformPoint(target);
        float alpha = Mathf.Atan2(localTarget.x, localTarget.z);
        float L = localTarget.magnitude;
        float delta = Mathf.Atan2(2.0f * wheelBase * Mathf.Sin(alpha) / L, 1.0f);

        return delta * Mathf.Rad2Deg;
    }

    void ApplySteering(float steeringAngle)
    {
        transform.Rotate(0, steeringAngle * Time.deltaTime * currentSpeed, 0); // Hızı ve dönüş açısını uygula
        transform.position += transform.forward * currentSpeed * Time.deltaTime; // Hızı uygula
    }
    void ApplyTest(float steeringAngle) 
    {
        frontRightCollider.steerAngle = steeringAngle;
        frontLeftCollider.steerAngle = steeringAngle;
        frontLeftCollider.motorTorque = frontRightCollider.motorTorque = torque;
        
    }
}
