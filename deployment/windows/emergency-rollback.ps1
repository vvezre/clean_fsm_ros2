# 文件作用：从 Windows 开发机远程触发树莓派执行紧急版本回滚。
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9.-]+$')]
    [string]$PiHost,

    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9._-]+$')]
    [string]$UserName
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$target = $UserName + '@' + $PiHost
$sshOptions = @('-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10')
& ssh @sshOptions $target 'sudo /usr/local/sbin/cleanbot-rollback'
if ($LASTEXITCODE -ne 0) {
    throw 'remote rollback failed'
}
