EAPI="5"

inherit eutils cmake-utils

DESCRIPTION="Find GPS reference clock configuration from set of frequencies"
HOMEPAGE="https://github.com/mhx/gpsdo-config"
SRC_URI="https://github.com/mhx/gpsdo-config/archive/v${PV}.tar.gz -> ${P}.tar.gz"

LICENSE="GPL-3"
SLOT="0"
KEYWORDS="amd64"
IUSE=""

DEPEND=">=dev-libs/boost-1.62.0"
RDEPEND="${DEPEND}"
