param([double]$Kp=4,[double]$Ki=4,[double]$Kd=0,[int]$Rpm=20,[int]$Seconds=12,[int[]]$Targets=@())
$ErrorActionPreference='Stop'
Add-Type -TypeDefinition @'
using System;
using System.IO;
using System.IO.Ports;
using System.Diagnostics;
using System.Collections.Generic;
using System.Globalization;
public class SpeedTune {
 public static void Run(double kp,double ki,double kd,int[] targets,int seconds,string output) {
  var p=new SerialPort("COM32",115200);p.ReadTimeout=200;p.WriteTimeout=1000;p.ReadBufferSize=262144;
  var samples=new List<double[]>();var pending=new List<byte>();var buf=new byte[65536];
  var watch=new Stopwatch();int stage=0;double lastValid=-1;Exception failure=null;
  try {
   p.Open();p.Write("steep,0D,");System.Threading.Thread.Sleep(300);p.DiscardInBuffer();
   p.Write(string.Format(CultureInfo.InvariantCulture,"SPID,{0}f,{1}f,{2}f,steep,{3}D,",kp,ki,kd,targets[0]));watch.Start();
   while(watch.Elapsed.TotalSeconds<seconds*targets.Length) {
    if(lastValid>=0 && watch.Elapsed.TotalSeconds-lastValid>2)throw new Exception("Telemetry interrupted; stopped");
    int next=(int)(watch.Elapsed.TotalSeconds/seconds);
    if(next>stage){Report(samples,stage,targets[stage],seconds,kp,ki,kd);stage=next;p.Write(string.Format(CultureInfo.InvariantCulture,"steep,{0}D,",targets[stage]));}
    int n;try{n=p.Read(buf,0,buf.Length);}catch(TimeoutException){continue;}
    for(int j=0;j<n;j++)pending.Add(buf[j]);
    int pos=0;
    while(pos+44<=pending.Count) {
     if(pending[pos+40]==0 && pending[pos+41]==0 && pending[pos+42]==128 && pending[pos+43]==127 && pos+88<=pending.Count && pending[pos+84]==0 && pending[pos+85]==0 && pending[pos+86]==128 && pending[pos+87]==127) {
      byte[] frame=pending.GetRange(pos,44).ToArray();double speed=BitConverter.ToSingle(frame,12);double iqref=BitConverter.ToSingle(frame,4);double id=BitConverter.ToSingle(frame,16);double iq=BitConverter.ToSingle(frame,20);
      bool valid=true;for(int channel=0;channel<10;channel++){float value=BitConverter.ToSingle(frame,channel*4);if(float.IsNaN(value)||float.IsInfinity(value))valid=false;}
      if(Math.Abs(iqref)>1501 || Math.Abs(id)>2100 || Math.Abs(iq)>2100 || BitConverter.ToSingle(frame,0)<0 || BitConverter.ToSingle(frame,0)>16383)valid=false;
      for(int channel=6;channel<=8;channel++)if(Math.Abs(BitConverter.ToSingle(frame,channel*4))>2100)valid=false;
      if(!valid){pos++;continue;}
      double limit=4000.0;if(stage>0)limit=Math.Max(limit,Math.Abs(targets[stage-1])*1.3+100);
      if(double.IsNaN(speed)||double.IsInfinity(speed)||Math.Abs(speed)>limit)throw new Exception("Invalid or excessive speed "+speed+" limit "+limit+"; stopped");
      lastValid=watch.Elapsed.TotalSeconds;
      samples.Add(new double[]{lastValid,speed,iqref,id,iq,BitConverter.ToSingle(frame,24),BitConverter.ToSingle(frame,28),BitConverter.ToSingle(frame,32),stage,targets[stage]});pos+=44;
     }else pos++;
    }
    if(pos>0)pending.RemoveRange(0,pos);
    if(pending.Count>4096)pending.RemoveRange(0,pending.Count-87);
   }
  } catch(Exception ex){failure=ex;} finally {if(p.IsOpen){try{p.Write("steep,0D,");}finally{p.Close();}}}
  using(var w=new StreamWriter(output)){w.WriteLine("time,speed,iq_ref,id,iq,ia,ib,ic,stage,target");foreach(var s in samples){w.WriteLine(string.Join(",",Array.ConvertAll(s,x=>x.ToString("R",CultureInfo.InvariantCulture))));}}
  Console.WriteLine("CSV="+output);if(failure!=null)throw failure;Report(samples,stage,targets[stage],seconds,kp,ki,kd);
 }
 static void Report(List<double[]> samples,int stage,int rpm,int seconds,double kp,double ki,double kd){
  var settled=samples.FindAll(x=>x[8]==stage && x[0]>=stage*seconds+seconds/2.0);if(settled.Count==0)throw new Exception("No settled samples");
  double sum=0,sq=0,min=double.MaxValue,max=double.MinValue,iqrefpeak=0,idpeak=0,iqpeak=0,phase=0;int sat=0;foreach(var s in settled){sum+=s[1];sq+=s[1]*s[1];min=Math.Min(min,s[1]);max=Math.Max(max,s[1]);iqrefpeak=Math.Max(iqrefpeak,Math.Abs(s[2]));idpeak=Math.Max(idpeak,Math.Abs(s[3]));iqpeak=Math.Max(iqpeak,Math.Abs(s[4]));for(int c=5;c<=7;c++)phase=Math.Max(phase,Math.Abs(s[c]));if(Math.Abs(s[2])>=1499)sat++;}
  double mean=sum/settled.Count;Console.WriteLine(string.Format(CultureInfo.InvariantCulture,"stage={0} P={1} I={2} D={3} target={4} mean={5:F3} std={6:F3} min={7:F3} max={8:F3} iqref_peak={9:F2} id_peak={10:F2} iq_peak={11:F2} phase_peak={12:F2} saturation={13:F1}%",stage,kp,ki,kd,rpm,mean,Math.Sqrt(Math.Max(0,sq/settled.Count-mean*mean)),min,max,iqrefpeak,idpeak,iqpeak,phase,100.0*sat/settled.Count));
 }
}
'@
$out=Join-Path (Get-Location) ('speed_tune_'+(Get-Date -Format 'yyyyMMdd_HHmmss')+'.csv')
if($Targets.Count -eq 0){$Targets=@($Rpm)}
[SpeedTune]::Run($Kp,$Ki,$Kd,$Targets,$Seconds,$out)
