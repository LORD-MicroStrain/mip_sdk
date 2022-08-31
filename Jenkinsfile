pipeline {
  agent none
  stages {
    stage('Build') {
      // Run the windows and linux builds in parallel
      parallel {
        stage('Windows x86') {
          agent { label 'windows10' }
          options { skipDefaultCheckout() }
          steps {
            cleanWs()
            checkout scm
            powershell """
                mkdir build_Win32
                cd build_Win32
                cmake .. `
                    -A "Win32" `
                    -DBUILD_DOCUMENTATION=ON `
                    -DBUILD_PACKAGE=ON
                cmake --build . -j
                cmake --build . --target package
                cmake --build . --target package_docs
            """
            archiveArtifacts artifacts: 'build_Win32/mipsdk_*'
          }
        }
        stage('Windows x64') {
          agent { label 'windows10' }
          options { skipDefaultCheckout() }
          steps {
            cleanWs()
            checkout scm
            powershell """
                mkdir build_x64
                cd build_x64
                cmake .. `
                    -A "x64" `
                    -DBUILD_PACKAGE=ON
                cmake --build . -j
                cmake --build . --target package
            """
            archiveArtifacts artifacts: 'build_x64/mipsdk_*'
          }
        }
        stage('Ubuntu amd64') {
          agent { label 'linux-amd64' }
          options { skipDefaultCheckout() }
          steps {
            cleanWs()
            checkout scm
            sh "cp /etc/pki/ca-trust/source/anchors/ZScaler.crt ./.devcontainer/extra_cas/"
            sh "./.devcontainer/docker_build.sh --os ubuntu --arch amd64"
            archiveArtifacts artifacts: 'build_ubuntu_amd64/mipsdk_*'
          }
        }
        stage('Centos amd64') {
          agent { label 'linux-amd64' }
          options { skipDefaultCheckout() }
          steps {
            cleanWs()
            checkout scm
            sh "cp /etc/pki/ca-trust/source/anchors/ZScaler.crt ./.devcontainer/extra_cas/"
            sh "./.devcontainer/docker_build.sh --os centos --arch amd64"
            archiveArtifacts artifacts: 'build_centos_amd64/mipsdk_*'
          }
        }
      }
    }
  }
}