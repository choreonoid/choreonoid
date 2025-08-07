# Launchpad PPA Setup Guide for Choreonoid

## Overview

This guide explains how to set up and use Launchpad PPA (Personal Package Archive) for distributing Choreonoid packages.

## Prerequisites

### 1. Launchpad Account

1. Create an account at https://launchpad.net
2. Set up your profile with your name and email

### 2. GPG Key Setup

#### Generate a GPG Key (if you don't have one)

```bash
# Generate a new GPG key
gpg --full-generate-key

# Select:
# - RSA and RSA (default)
# - Key size: 4096 bits
# - Expiration: 2 years (recommended)
# - Your real name and email (must match Launchpad account)
```

#### List Your Keys

```bash
# List secret keys with long format
gpg --list-secret-keys --keyid-format LONG

# Example output:
# sec   rsa4096/ABCD1234EFGH5678 2024-01-01 [SC]
#       Key fingerprint = 1234 5678 9ABC DEF0 1234  5678 9ABC DEF0 1234 5678
# uid                 [ultimate] Your Name <your.email@example.com>
```

The key ID is `ABCD1234EFGH5678` in this example.

#### Upload Key to Ubuntu Keyserver

```bash
# Upload your public key
gpg --keyserver keyserver.ubuntu.com --send-keys YOUR_KEY_ID
```

#### Add Key to Launchpad

1. Go to https://launchpad.net/~YOUR_USERNAME/+editpgpkeys
2. Copy your key fingerprint:
   ```bash
   gpg --fingerprint YOUR_KEY_ID
   ```
3. Paste the fingerprint in Launchpad
4. Launchpad will send an encrypted email
5. Decrypt and follow the confirmation link:
   ```bash
   # Save the email to a file, then:
   gpg --decrypt email.txt
   ```

### 3. Create a PPA

1. Go to https://launchpad.net/~YOUR_USERNAME
2. Click "Create a new PPA"
3. Choose a name (e.g., "choreonoid" or "choreonoid-dev")
4. Add description
5. Click "Create PPA"

### 4. Install Required Tools

```bash
sudo apt update
sudo apt install dput devscripts git-buildpackage
```

## Using the Upload Script

### Basic Usage

```bash
# Upload to PPA for all supported distributions
./debian/upload-to-ppa.sh -p YOUR_USERNAME/PPA_NAME -d all -k YOUR_KEY_ID

# Example:
./debian/upload-to-ppa.sh -p johndoe/choreonoid -d all -k ABCD1234EFGH5678
```

### Options

| Option | Description | Example |
|--------|-------------|---------|
| `-p, --ppa` | PPA name | `johndoe/choreonoid` |
| `-d, --distro` | Distribution(s) | `jammy`, `noble`, `all` |
| `-k, --key` | GPG key ID | `ABCD1234EFGH5678` |
| `-s, --suffix` | Version suffix | `ppa1`, `ppa2` |
| `-f, --force` | Force upload | |
| `-n, --dry-run` | Test without uploading | |

### Examples

#### Test Run (Dry Run)
```bash
./debian/upload-to-ppa.sh -p johndoe/choreonoid -d jammy -k ABCD1234 --dry-run
```

#### Upload for Specific Distribution
```bash
./debian/upload-to-ppa.sh -p johndoe/choreonoid -d noble -k ABCD1234
```

#### Upload with Custom Version Suffix
```bash
./debian/upload-to-ppa.sh -p johndoe/choreonoid -d all -k ABCD1234 -s ppa2
```

## Version Numbering

The script generates versions in this format:
```
2.3.0~git20240814.abc1234~jammy~ppa1
│     │            │        │     └── PPA revision
│     │            │        └──────── Distribution
│     │            └───────────────── Git commit hash
│     └────────────────────────────── Git snapshot date
└──────────────────────────────────── Base version
```

This ensures:
- Newer git commits have higher version numbers
- Distribution-specific builds don't interfere
- PPA revisions allow re-uploads with fixes

## Workflow

### 1. Development Workflow

```bash
# 1. Make changes to the code
git add .
git commit -m "Your changes"

# 2. Test locally first
./debian/build-packages.sh -d jammy -l

# 3. If successful, upload to PPA
./debian/upload-to-ppa.sh -p YOUR_USERNAME/choreonoid -d all -k YOUR_KEY_ID
```

### 2. Release Workflow

```bash
# 1. Update changelog for release
dch -v 2.3.0-1 "New release"
dch -r  # Finalize changelog

# 2. Commit and tag
git add debian/changelog
git commit -m "Release version 2.3.0-1"
git tag v2.3.0

# 3. Upload to PPA
./debian/upload-to-ppa.sh -p YOUR_USERNAME/choreonoid -d all -k YOUR_KEY_ID
```

## Monitoring Builds

After uploading:

1. **Check Upload Status**
   - You'll receive an email about acceptance/rejection
   - Usually takes 5-10 minutes

2. **Monitor Build Progress**
   - Go to: https://launchpad.net/~YOUR_USERNAME/+archive/ubuntu/PPA_NAME
   - Click on package name to see build status
   - Builds typically take 15-60 minutes

3. **Build States**
   - **Pending**: Waiting in queue
   - **Building**: Currently compiling
   - **Successfully built**: Ready for use
   - **Failed to build**: Check build log for errors

## Installing from PPA

Once packages are built:

```bash
# Add the PPA
sudo add-apt-repository ppa:YOUR_USERNAME/PPA_NAME
sudo apt update

# Install Choreonoid
sudo apt install choreonoid
```

## Troubleshooting

### Common Issues

#### 1. GPG Key Not Found
```
Error: GPG key XXXXX not found
```
**Solution**: Ensure key is in your keyring:
```bash
gpg --list-secret-keys --keyid-format LONG
```

#### 2. Already Uploaded
```
Error: File choreonoid_XXX.dsc already uploaded
```
**Solution**: Use `-f` flag to force, or increment PPA suffix:
```bash
./debian/upload-to-ppa.sh -p ... -s ppa2 -f
```

#### 3. Build Failures on Launchpad
Check the build log:
1. Go to your PPA page
2. Click on the failed build
3. View "buildlog"
4. Common issues:
   - Missing dependencies: Add to `debian/control`
   - Compilation errors: Fix and re-upload with new PPA suffix

#### 4. GPG Signing Failed
```
Error: Secret key not available
```
**Solution**: Specify correct key ID:
```bash
gpg --list-secret-keys
./debian/upload-to-ppa.sh -k CORRECT_KEY_ID ...
```

### Debugging Tips

1. **Use Dry Run First**
   ```bash
   ./debian/upload-to-ppa.sh ... --dry-run
   ```

2. **Check Generated Files**
   ```bash
   ls -la ../ppa-build-area/
   ```

3. **Verify Source Package**
   ```bash
   dpkg-source -x ../ppa-build-area/*.dsc
   ```

4. **Test Build Locally**
   ```bash
   ./debian/build-packages.sh -d jammy
   ```

## Best Practices

1. **Test Locally First**: Always use `build-packages.sh` before uploading
2. **Use Dry Run**: Test upload process with `--dry-run`
3. **Version Management**: Increment PPA suffix for fixes
4. **Monitor Emails**: Launchpad sends important notifications
5. **Document Changes**: Update changelog meaningfully
6. **Clean Commits**: Ensure git repository is clean before uploading

## Additional Resources

- [Launchpad Help](https://help.launchpad.net/Packaging/PPA)
- [Ubuntu Packaging Guide](https://packaging.ubuntu.com/)
- [Debian Policy Manual](https://www.debian.org/doc/debian-policy/)
- [GPG Best Practices](https://help.ubuntu.com/community/GnuPrivacyGuardHowto)

---

*Last updated: 2025-08-14*