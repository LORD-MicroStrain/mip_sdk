// Utility function for getting the real branch name even in a pull request
def branchName() {
    if (env.CHANGE_BRANCH) {
        return env.CHANGE_BRANCH
    } else {
        return env.BRANCH_NAME
    }
}

// Utility function for checking out the repo since we want all the branches and tags
def checkoutRepo() {
    // Clean the workspace
    cleanWs()

    // Handles both PRs and branches
    script {
        BRANCH_NAME_REAL = branchName();
    }

    // Checkout the code from github
    checkout([  $class: 'GitSCM',
        branches: [
            [name: 'refs/heads/' + BRANCH_NAME_REAL]
        ],
        userRemoteConfigs: [[credentialsId: 'Github_User_And_Token', url: 'https://github.com/LORD-MicroStrain/mip_sdk.git']],
        extensions: [
        ]
    ])

  // Set the branch name
  env.setProperty('BRANCH_NAME', branchName())
}

// Utility function to build the MIP SDK on linux
// This script requires BUILD_OS and BUILD_ARCH to have been set in the environment before it is called
def buildMipSdkLinux() {
    // Build the docker image and MIP SDK
    sh """
        ./.devcontainer/docker_build_image.sh --os ${BUILD_OS} --arch ${BUILD_ARCH}
        ./.devcontainer/docker_shell.sh --os ${BUILD_OS} --arch ${BUILD_ARCH} " \
            cmake \
                -B build_${BUILD_OS}_${BUILD_ARCH} \
                -DMICROSTRAIN_BUILD_EXAMPLES=ON \
                -DMICROSTRAIN_BUILD_PACKAGE=ON \
                -DMICROSTRAIN_BUILD_TESTS=ON \
                -DCMAKE_BUILD_TYPE=RELEASE; \
            cmake \
                --build build_${BUILD_OS}_${BUILD_ARCH} \
                --target package \
                -j $(nproc); \
            ctest \
                --test-dir build_${BUILD_OS}_${BUILD_ARCH} \
                -C Release \
                --verbose \
                --output-on-failure \
                --output-junit unit_test_results.xml \
                --parallel $(nproc); \
        "
    """

    // Archive the artifacts and save the unit test results 
    dir("build_${BUILD_OS}_${BUILD_ARCH}") {
        archiveArtifacts artifacts: "mipsdk_*"
        junit testResults: "unit_test_results.xml", allowEmptyResults: false
    }
}

pipeline {
    agent none
    options {
        // Set a timeout for the whole pipeline. The timer starts when the project is queued
        timeout(time: 1, unit: 'HOURS')
        // Only keep this number of builds for the job
        buildDiscarder(logRotator(numToKeepStr: "10"))
        copyArtifactPermission('*')
    }
    stages {
        stage('Build') {
            // Run all the builds in parallel
            parallel {
                stage('Documentation') {
                    agent {
                        label 'linux-amd64'
                    }
                    environment {
                        BUILD_OS = "ubuntu"
                        BUILD_ARCH = "amd64"
                    }
                    options {
                        skipDefaultCheckout()
                        timeout(time: 5, activity: true, unit: 'MINUTES')
                    }
                    steps {
                        script {
                            checkoutRepo()
                            sh """
                                ./.devcontainer/docker_build_image.sh --os ${BUILD_OS} --arch ${BUILD_ARCH}
                                ./.devcontainer/docker_shell.sh --os ${BUILD_OS} --arch ${BUILD_ARCH} " \
                                    cmake \
                                        -B build_docs \
                                        -DMICROSTRAIN_BUILD_DOCUMENTATION=ON \
                                        -DMICROSTRAIN_BUILD_DOCUMENTATION_QUIET=OFF \
                                        -DCMAKE_BUILD_TYPE=RELEASE; \ \
                                    cmake \
                                        --build build_docs \
                                        --target package_docs \
                                        -j $(nproc)
                                "
                            """
                            archiveArtifacts artifacts: "build_docs/mipsdk_*"
                        }
                    }
                }
                stage('Windows x86') {
                    agent {
                        label 'windows10'
                    }
                    environment {
                        BUILD_ARCH = "Win32"
                    }
                    options {
                        skipDefaultCheckout()
                        timeout(time: 5, activity: true, unit: 'MINUTES')
                    }
                    steps {
                        script {
                            checkoutRepo()
                            dir("build_${BUILD_ARCH}") {
                                powershell """
                                    cmake .. `
                                        -A "${BUILD_ARCH}" `
                                        -DMICROSTRAIN_BUILD_EXAMPLES=ON `
                                        -DMICROSTRAIN_BUILD_PACKAGE=ON `
                                        -DMICROSTRAIN_BUILD_TESTS=ON
                                    cmake --build . --config Release --parallel \$env:NUMBER_OF_PROCESSORS
                                    cmake --build . --config Release --parallel \$env:NUMBER_OF_PROCESSORS --target package

                                """
                                archiveArtifacts artifacts: "mipsdk_*"
                                powershell """
                                    ctest `
                                        -C Release `
                                        --verbose `
                                        --output-on-failure `
                                        --output-junit unit_test_results.xml `
                                        --parallel \$env:NUMBER_OF_PROCESSORS
                                """
                                junit testResults: "unit_test_results.xml", allowEmptyResults: false
                            }
                        }
                    }
                }
                stage('Windows x64') {
                    agent {
                        label 'windows10'
                    }
                    environment {
                        BUILD_ARCH = "x64"
                    }
                    options {
                        skipDefaultCheckout()
                        timeout(time: 5, activity: true, unit: 'MINUTES')
                    }
                    steps {
                        script {
                            checkoutRepo()
                            dir("build_${BUILD_ARCH}") {
                                powershell """
                                    cmake .. `
                                        -A "${BUILD_ARCH}" `
                                        -DMICROSTRAIN_BUILD_EXAMPLES=ON `
                                        -DMICROSTRAIN_BUILD_PACKAGE=ON `
                                        -DMICROSTRAIN_BUILD_TESTS=ON
                                    cmake --build . --config Release --parallel \$env:NUMBER_OF_PROCESSORS
                                    cmake --build . --config Release --parallel \$env:NUMBER_OF_PROCESSORS --target package

                                """
                                archiveArtifacts artifacts: "mipsdk_*"
                                powershell """
                                    ctest `
                                        -C Release `
                                        --verbose `
                                        --output-on-failure `
                                        --output-junit unit_test_results.xml `
                                        --parallel \$env:NUMBER_OF_PROCESSORS
                                """
                                junit testResults: "unit_test_results.xml", allowEmptyResults: false
                            }
                        }
                    }
                }
                stage('Ubuntu amd64') {
                    agent {
                        label 'linux-amd64'
                    }
                    environment {
                        BUILD_OS = "ubuntu"
                        BUILD_ARCH = "amd64"
                    }
                    options {
                        skipDefaultCheckout()
                        timeout(time: 5, activity: true, unit: 'MINUTES')
                    }
                    steps {
                        script {
                            checkoutRepo()
                            buildMipSdkLinux()
                        }
                    }
                }
                stage('Ubuntu arm64') {
                    agent {
                        label 'linux-arm64'
                    }
                    environment {
                        BUILD_OS = "ubuntu"
                        BUILD_ARCH = "arm64v8"
                    }
                    options {
                        skipDefaultCheckout()
                        timeout(time: 5, activity: true, unit: 'MINUTES')
                    }
                    steps {
                        script {
                            checkoutRepo()
                            buildMipSdkLinux()
                        }
                    }
                }
                stage('Ubuntu arm32') {
                    agent {
                        label 'linux-arm64'
                    }
                    environment {
                        BUILD_OS = "ubuntu"
                        BUILD_ARCH = "arm32v7"
                    }
                    options {
                        skipDefaultCheckout()
                        timeout(time: 5, activity: true, unit: 'MINUTES')
                    }
                    steps {
                        script {
                            checkoutRepo()
                            buildMipSdkLinux()
                        }
                    }
                }
//                 stage("Mac M2") {
//                     agent {
//                         label 'mac-m2'
//                     }
//                     environment {
//                         BUILD_OS = "mac"
//                         BUILD_ARCH = "arm64"
//                     }
//                     options {
//                         skipDefaultCheckout()
//                         timeout(time: 5, activity: true, unit: 'MINUTES')
//                     }
//                     steps {
//                         script {
//                             checkoutRepo()
//                             dir("build_${BUILD_OS}_${BUILD_ARCH}") {
//                                 sh '''
//                                     cmake .. `
//                                         -DMICROSTRAIN_BUILD_EXAMPLES=ON `
//                                         -DMICROSTRAIN_BUILD_PACKAGE=ON `
//                                         -DMICROSTRAIN_BUILD_TESTS=ON
//                                     cmake --build . --parallel $(sysctl -n hw.ncpu)
//                                     cmake --build . --parallel $(sysctl -n hw.ncpu) --target package
//                                 '''
//                                 archiveArtifacts artifacts: "mipsdk_*"
//                                 sh '''
//                                     ctest `
//                                         -C Release `
//                                         --verbose `
//                                         --output-on-failure `
//                                         --output-junit unit_test_results.xml `
//                                         --parallel $(sysctl -n hw.ncpu)
//                                 '''
//                                 junit testResults: "unit_test_results.xml", allowEmptyResults: false
//                             }
//                         }
//                     }
//                 }
//                 stage("Mac Intel") {
//                     agent {
//                         label 'mac-intel'
//                     }
//                     environment {
//                         BUILD_OS = "mac"
//                         BUILD_ARCH = "intel"
//                     }
//                     options {
//                         skipDefaultCheckout()
//                         timeout(time: 5, activity: true, unit: 'MINUTES')
//                     }
//                     steps {
//                         script {
//                             checkoutRepo()
//                             dir("build_${BUILD_OS}_${BUILD_ARCH}") {
//                                 sh '''
//                                     cmake .. `
//                                         -DMICROSTRAIN_BUILD_EXAMPLES=ON `
//                                         -DMICROSTRAIN_BUILD_PACKAGE=ON `
//                                         -DMICROSTRAIN_BUILD_TESTS=ON
//                                     cmake --build . --parallel $(sysctl -n hw.ncpu)
//                                     cmake --build . --parallel $(sysctl -n hw.ncpu) --target package
//                                 '''
//                                 archiveArtifacts artifacts: "mipsdk_*"
//                                 sh '''
//                                     ctest `
//                                         -C Release `
//                                         --verbose `
//                                         --output-on-failure `
//                                         --output-junit unit_test_results.xml `
//                                         --parallel $(sysctl -n hw.ncpu)
//                                 '''
//                                 junit testResults: "unit_test_results.xml", allowEmptyResults: false
//                             }
//                         }
//                     }
//                 }
            }
        }
    }
    post {
        success {
            script {
                if (BRANCH_NAME && BRANCH_NAME == 'develop') {
                    node("linux-amd64") {
                        dir("/tmp/mip_sdk_${env.BRANCH_NAME}_${currentBuild.number}") {
                            copyArtifacts(projectName: "${env.JOB_NAME}", selector: specific("${currentBuild.number}"));
                            withCredentials([string(credentialsId: 'Github_Token', variable: 'GH_TOKEN')]) {
                                sh '''
                                    # Release to github
                                    "${WORKSPACE}/scripts/release.sh" \
                                        --artifacts "$(find "$(pwd)" -type f)" \
                                        --target "${BRANCH_NAME}" \
                                        --release "latest" \
                                        --docs-zip "$(find "$(pwd)" -type f -name "mipsdk_*_Documentation.zip" | sort | uniq)" \
                                        --generate-notes
                                '''
                            }
                        }
                    }
                } else if (BRANCH_NAME && BRANCH_NAME == 'master') {
                    node("linux-amd64") {
                        dir("/tmp/mip_sdk_${env.BRANCH_NAME}_${currentBuild.number}") {
                            copyArtifacts(projectName: "${env.JOB_NAME}", selector: specific("${currentBuild.number}"));
                            withCredentials([string(credentialsId: 'Github_Token', variable: 'GH_TOKEN')]) {
                                sh '''
                                    # Release to the latest version if the master commit matches up with the commit of that version
                                    if (cd "${WORKSPACE}" && git describe --exact-match --tags HEAD &> /dev/null); then
                                        # Publish a release
                                        ${WORKSPACE}/scripts/release.sh" \
                                            --artifacts "$(find "$(pwd)" -type f)" \
                                            --target "${BRANCH_NAME}" \
                                            --release "$(cd ${WORKSPACE} && git describe --exact-match --tags HEAD)" \
                                            --docs-zip "$(find "$(pwd)" -type f -name "mipsdk_*_Documentation.zip" | sort | uniq)"
                                    else
                                        echo "Not releasing from ${BRANCH_NAME} since the current commit does not match the latest released version commit"
                                    fi
                                '''
                            }
                        }
                    }
                }
            }
        }
    }
}
