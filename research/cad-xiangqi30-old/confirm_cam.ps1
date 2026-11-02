# 用户已明确同意仅本次参考总装副本不保留CAM数据；仅处理该实例的CAM模态框。
# 使用窗口消息调用按钮，不发送鼠标或键盘输入、不激活其他窗口。
param([uint32]$CadProcessId = 66828)
Add-Type @'
using System;using System.Text;using System.Collections.Generic;using System.Runtime.InteropServices;
public class ConfirmOwnedCam {
 public delegate bool F(IntPtr w,IntPtr p);
 [DllImport("user32.dll")]static extern bool EnumWindows(F f,IntPtr p);
 [DllImport("user32.dll")]static extern bool EnumChildWindows(IntPtr w,F f,IntPtr p);
 [DllImport("user32.dll")]static extern uint GetWindowThreadProcessId(IntPtr w,out uint p);
 [DllImport("user32.dll",CharSet=CharSet.Unicode)]static extern int GetWindowText(IntPtr w,StringBuilder b,int n);
 [DllImport("user32.dll")]static extern bool IsWindowVisible(IntPtr w);
 [DllImport("user32.dll")]static extern int GetDlgCtrlID(IntPtr w);
 [DllImport("user32.dll")]static extern bool PostMessage(IntPtr w,uint m,IntPtr p,IntPtr l);
 static string Text(IntPtr w){var b=new StringBuilder(2048);GetWindowText(w,b,2048);return b.ToString();}
 public static string One(uint target){var ds=new List<IntPtr>();EnumWindows((w,p)=>{uint pid;GetWindowThreadProcessId(w,out pid);string t=Text(w);if(pid==target&&IsWindowVisible(w)&&(t=="SOLIDWORKS CAM 错误"||t=="SOLIDWORKS CAM 警告"))ds.Add(w);return true;},IntPtr.Zero);
 if(ds.Count==0)return "No CAM dialogs";
 IntPtr d=ds.Find(w=>Text(w)=="SOLIDWORKS CAM 错误");if(d==IntPtr.Zero)d=ds[0];
 bool err=Text(d)=="SOLIDWORKS CAM 错误";IntPtr button=IntPtr.Zero;string body="";
 EnumChildWindows(d,(w,p)=>{string t=Text(w);body+=t; if(t==(err?"确定":"是(&Y)"))button=w;return true;},IntPtr.Zero);
 if(button==IntPtr.Zero||!body.Contains("轻化零件")||!body.Contains("CAM 数据"))throw new Exception("Unexpected dialog; no action");
 if(!PostMessage(d,0x111,new IntPtr(GetDlgCtrlID(button)),button))throw new Exception("PostMessage failed");
 return "Confirmed "+Text(d)+" via button "+GetDlgCtrlID(button)+"; dialogs before="+ds.Count;
 }
}
'@
[ConfirmOwnedCam]::One($CadProcessId)
