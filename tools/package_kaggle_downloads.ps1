param(
    [string]$Corpus = 'D:\data\lunabot-datasets\canonical-20261110',
    [string]$Repo = 'D:\lunabot-v4',
    [string]$Output = 'D:\data\lunabot-kaggle\kaggle-p502-new-datasets.zip'
)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.IO.Compression.FileSystem
$audit = Get-Content -Raw -LiteralPath (Join-Path $Corpus 'audit.json') | ConvertFrom-Json
if ($audit.result -ne 'PASS') { throw 'Canonical corpus audit did not pass' }
foreach ($name in @('manifest.csv', 'sources.json', 'splits.json')) {
    if ((Get-FileHash -LiteralPath (Join-Path $Corpus $name) -Algorithm SHA256).Hash.ToLowerInvariant() -ne $audit.configuration_sha256.$name) {
        throw "Corpus changed after audit: $name"
    }
}
[IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($Output)) | Out-Null
$rows = @(Import-Csv -LiteralPath (Join-Path $Corpus 'manifest.csv'))
$files = [Collections.Generic.Dictionary[string,string]]::new()
$hasher = [Security.Cryptography.SHA256]::Create()
foreach ($row in $rows) {
    foreach ($field in @('image', 'mask')) {
        $linuxPath = $row.$field
        if (-not $linuxPath.StartsWith('/mnt/d/')) { throw "Corpus file is outside D: $linuxPath" }
        $localPath = 'D:\' + $linuxPath.Substring(7).Replace('/', '\')
        $key = [Convert]::ToHexString($hasher.ComputeHash([Text.Encoding]::UTF8.GetBytes($linuxPath))).ToLowerInvariant()
        $relative = 'data/' + $key + [IO.Path]::GetExtension($localPath)
        $files[$relative] = $localPath
        $row.$field = $relative
    }
}
$temporary = [IO.Path]::ChangeExtension($Output, '.partial.zip')
if (Test-Path -LiteralPath $temporary) { throw "Partial package exists: $temporary" }
if (Test-Path -LiteralPath $Output) { throw "Package already exists: $Output" }
$zip = [IO.Compression.ZipFile]::Open($temporary, [IO.Compression.ZipArchiveMode]::Create)
try {
    $manifestEntry = $zip.CreateEntry('manifest.csv')
    $writer = [IO.StreamWriter]::new($manifestEntry.Open(), [Text.UTF8Encoding]::new($false))
    try { $writer.Write(($rows | ConvertTo-Csv -NoTypeInformation) -join "`n"); $writer.Write("`n") } finally { $writer.Dispose() }
    $extras = [ordered]@{
        'config/sources.json' = (Join-Path $Corpus 'sources.json')
        'config/splits.json' = (Join-Path $Corpus 'splits.json')
        'config/materialization.json' = (Join-Path $Corpus 'materialization.json')
        'config/audit.json' = (Join-Path $Corpus 'audit.json')
        'config/archive-provenance.json' = 'D:\data\lunabot-datasets\staging.json'
        'config/inherited-corpus-audit.json' = (Join-Path $Repo 'evidence\runs\20261107-p502\P5-02\corpus-audit-reviewed-v2.json')
        'config/gates-frozen.json' = (Join-Path $Repo 'evidence\runs\20261107-p502\P5-02\gates-frozen.json')
        'config/classes.json' = (Join-Path $Repo 'config\perception\classes.json')
        'README-project.md' = (Join-Path $Repo 'docs\d-drive-training.md')
        'weights/resnet34-b627a593.pth' = 'D:\data\lunabot-torch-cache\hub\checkpoints\resnet34-b627a593.pth'
        'sources/roboflow-README.dataset.txt' = 'D:\data\lunabot-datasets\coco\Dataset_coco\README.dataset.txt'
        'sources/roboflow-data.yaml' = 'D:\data\lunabot-datasets\yolo\Dataset_yolo\data.yaml'
        'sources/ai4mars-info.md' = 'D:\data\lunabot-datasets\20261110-012024-584472\ai4mars-001\ai4mars-dataset-merged-0.6\info.md'
        'sources/ai4mars-label_keys.json' = 'D:\data\lunabot-datasets\20261110-012024-584472\ai4mars-001\ai4mars-dataset-merged-0.6\label_keys.json'
    }
    foreach ($name in @('train_downloaded_resnet.py', 'evaluate_downloaded_resnet.py', 'train_perception.py', 'perception_metrics.py')) {
        $extras['tools/' + $name] = Join-Path $Repo ('tools\' + $name)
    }
    foreach ($item in $extras.GetEnumerator()) {
        [IO.Compression.ZipFileExtensions]::CreateEntryFromFile($zip, $item.Value, $item.Key, [IO.Compression.CompressionLevel]::NoCompression) | Out-Null
    }
    $packed = 0
    foreach ($relative in ($files.Keys | Sort-Object)) {
        [IO.Compression.ZipFileExtensions]::CreateEntryFromFile($zip, $files[$relative], $relative, [IO.Compression.CompressionLevel]::NoCompression) | Out-Null
        $packed++
        if ($packed % 2000 -eq 0) { Write-Output "Packed $packed/$($files.Count) files" }
    }
} finally { $zip.Dispose(); $hasher.Dispose() }
Move-Item -LiteralPath $temporary -Destination $Output
$summary = @{ archive = $Output; records = $rows.Count; files = $files.Count; bytes = ([IO.FileInfo]::new($Output)).Length; sha256 = (Get-FileHash -LiteralPath $Output -Algorithm SHA256).Hash.ToLowerInvariant() }
$summary | ConvertTo-Json | Set-Content -LiteralPath ([IO.Path]::ChangeExtension($Output, '.json'))
Copy-Item -LiteralPath (Join-Path $Repo 'notebooks\p502-new-datasets-kaggle.ipynb') -Destination ([IO.Path]::GetDirectoryName($Output))
$summary | ConvertTo-Json
