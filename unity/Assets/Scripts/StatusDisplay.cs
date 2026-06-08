using UnityEngine;
using TMPro;

public class StatusDisplay : MonoBehaviour
{
    [SerializeField] private MotorControllerUI motorUI;
    [SerializeField] private TMP_Text posText;
    [SerializeField] private TMP_Text speedText;
    [SerializeField] private TMP_Text currentText;
    [SerializeField] private TMP_Text torqueText;
    [SerializeField] private TMP_Text tempText;
    [SerializeField] private TMP_Text errorText;

    void Update()
    {
        if (motorUI == null) return;
        MotorFeedback fb = motorUI.LastFeedback;

        posText.text = $"Pos: {fb.position:F3} rad";
        speedText.text = $"Speed: {fb.speed:F3} rad/s";
        currentText.text = $"Current: {fb.current:F3} A";
        torqueText.text = $"Torque: {fb.torque:F3} Nm";
        tempText.text = $"Temp: {fb.temperature:F1} °C";
        errorText.text = $"Error: {fb.errorMsg}";
        errorText.color = fb.errorCode == 0 ? Color.black : Color.red;
    }
}
