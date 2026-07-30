[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9.-]+$')]
    [string]$PiHost,

    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9._-]+$')]
    [string]$UserName,

    [Parameter(Mandatory = $true)]
    [string]$ReleaseDirectory,

    [Parameter(Mandatory = $true)]
    [string]$PublicKey,

    [switch]$Start
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$releasePath = (Resolve-Path -LiteralPath $ReleaseDirectory).Path
$publicKeyPath = (Resolve-Path -LiteralPath $PublicKey).Path
$deploymentPath = (Resolve-Path -LiteralPath (
    Join-Path $PSScriptRoot '..'
)).Path
if (-not (Test-Path -LiteralPath (Join-Path $releasePath 'install\setup.bash') -PathType Leaf)) {
    throw 'ReleaseDirectory must contain install/setup.bash'
}

$remoteRoot = '/tmp/cleanbot-bootstrap-' + [Guid]::NewGuid().ToString('N')
$target = $UserName + '@' + $PiHost
$sshOptions = @('-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10')

try {
    & ssh @sshOptions $target "install -d -m 0700 '$remoteRoot'"
    if ($LASTEXITCODE -ne 0) { throw 'failed to create remote staging directory' }

    & scp @sshOptions -r $deploymentPath ($target + ':' + $remoteRoot + '/deployment')
    if ($LASTEXITCODE -ne 0) { throw 'failed to upload deployment files' }
    & scp @sshOptions -r $releasePath ($target + ':' + $remoteRoot + '/release')
    if ($LASTEXITCODE -ne 0) { throw 'failed to upload initial release' }
    & scp @sshOptions $publicKeyPath ($target + ':' + $remoteRoot + '/public.pem')
    if ($LASTEXITCODE -ne 0) { throw 'failed to upload update public key' }

    $startArgument = if ($Start) { ' --start' } else { '' }
    $installCommand = "sudo bash '$remoteRoot/deployment/install/install.sh' " +
        "'$remoteRoot/release' '$remoteRoot/public.pem'$startArgument"
    & ssh @sshOptions $target $installCommand
    if ($LASTEXITCODE -ne 0) { throw 'remote bootstrap failed' }
}
finally {
    & ssh @sshOptions $target "rm -rf -- '$remoteRoot'" | Out-Null
}
