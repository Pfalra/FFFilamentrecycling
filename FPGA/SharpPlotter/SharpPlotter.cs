using ScottPlot;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace SharpPlotter
{
    public partial class SharpPlot : Form
    {
        private SerialPort Port = null;
        private Thread MainThread;
        private System.Windows.Forms.Timer UpdateTimer;

        private double[] signalX = new double[8192];
        private double[] signalY = new double[8192];
        private byte[] PayloadBuffer = new byte[8192];
        private int PayloadUsed = 0;
        private double SmoothedBaseline = 1.0f;
        private double SmoothedWidth = 0.0f;
        private double SmoothedX = 0.0f;

        public SharpPlot()
        {
            InitializeComponent();

            formsPlot1.Plot.XLabel("Pixel");
            formsPlot1.Plot.YLabel("Amplitude");
            formsPlot1.Plot.Title("CMOS plot");
            formsPlot1.Plot.Style(Style.Gray2);
        }

        private void btnConnect_Click(object sender, EventArgs e)
        {
            Port = new SerialPort(txtPort.Text, 500000);
            Port.Open();

            MainThread = new Thread(MainFunc);
            MainThread.Start();

            UpdateTimer = new System.Windows.Forms.Timer();
            UpdateTimer.Interval = 15;
            UpdateTimer.Tick += UpdateTimer_Tick;
            UpdateTimer.Start();
        }

        private void UpdateTimer_Tick(object sender, EventArgs e)
        {
            int used = PayloadUsed;
            if (used < 100)
            {
                return;
            }

            if(signalX.Length != used)
            {
                Array.Resize(ref signalX, used);
                Array.Resize(ref signalY, used);
            }

            for (int pos = 0; pos < used; pos++)
            {
                signalX[pos] = pos;
                signalY[pos] = PayloadBuffer[pos] / 255.0f;
            }

            double max = signalY.Skip(64).Reverse().Skip(32).Max();
            double min = signalY.Skip(64).Reverse().Skip(32).Min();
            double baseLine = min + (max - min) / 2;
            SmoothedBaseline = (10 * SmoothedBaseline + baseLine) / 11;

            double dH = SmoothedBaseline + 0.01;
            double dL = SmoothedBaseline - 0.01;
            bool above = false;
            int start = 0;
            int end = 0;
            int width = 0;
            int posX = 0;
            for (int pos = 0; pos < used; pos++)
            {
                if(!above && signalY[pos] > dH)
                {
                    start = pos;
                    above = true;
                }
                if (above && signalY[pos] < dL)
                {
                    end = pos;
                    above = false;
                    if(end-start > width)
                    {
                        width = end - start;
                        posX = start + (end - start) / 2;
                    }
                }
            }

            SmoothedWidth = (10 * SmoothedWidth + width) / 11;
            SmoothedX = (10 * SmoothedX + posX) / 11;

            formsPlot1.Plot.Clear();
            formsPlot1.Plot.PlotFillAboveBelow(signalX, signalY, baseline: SmoothedBaseline);
            formsPlot1.Plot.PlotText("Width: " + (SmoothedWidth / 128.0f / 2.24f * 1.75f).ToString("0.00") + " mm", SmoothedX - 20, 0.5f);
            formsPlot1.Plot.SetAxisLimits(0, 1096, 0.1, 0.75f);
            formsPlot1.Render();
        }

        protected override void OnClosing(CancelEventArgs e)
        {
            MainThread?.Abort();
            base.OnClosing(e);
        }

        private void MainFunc()
        {
            char[] syncBuf = new char[4];
            byte[] serialReadBuf = new byte[2048];
            int readPos = 0;

            while (true)
            {
                int read = Port.Read(serialReadBuf, 0, serialReadBuf.Length);

                for (int byteNum = 0; byteNum < read; byteNum++)
                {
                    byte ch = (byte)serialReadBuf[byteNum];

                    for (int pos = 0; pos < 3; pos++)
                    {
                        syncBuf[0 + pos] = syncBuf[1 + pos];
                    }
                    syncBuf[3] = (char)ch;

                    if (syncBuf[0] == 'S' && syncBuf[1] == 'Y' && syncBuf[2] == 'N' && syncBuf[3] == 'C')
                    {
                        if (readPos > 4)
                        {
                            PayloadUsed = readPos - 4;
                        }
                        readPos = 0;
                    }
                    else if (readPos < PayloadBuffer.Length)
                    {
                        PayloadBuffer[readPos] = ch;
                        readPos++;
                    }
                }
            }
        }
    }
}

